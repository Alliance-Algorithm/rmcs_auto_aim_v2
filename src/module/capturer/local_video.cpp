#include "local_video.hpp"

#include <filesystem>
#include <thread>

#include <opencv2/videoio.hpp>

#include "utility/image/image.details.hpp"

using namespace rmcs::cap;

struct LocalVideo::Impl {
    Config config;

    using Clock = std::chrono::steady_clock;

    std::optional<cv::VideoCapture> capturer;

    std::chrono::nanoseconds interval_duration { 0 };
    Clock::time_point last_read_time { Clock::now() };

    /**
     * @brief 根据目标帧率设置内部帧间隔时长。
     *
     * 将内部的 interval_duration 设置为对应于 hz（赫兹）的帧间隔（以纳秒为单位）。当 hz 小于或等于 0 时，将间隔设为 0 纳秒（立即读取，不延迟）。
     *
     * @param hz 目标帧率（Hz）。大于 0 时按 1/hz 计算间隔，非正值表示禁用间隔（0 纳秒）。
     */
    auto set_framerate_interval(double hz) noexcept -> void {
        if (hz > 0) {
            interval_duration =
                std::chrono::nanoseconds(static_cast<long long>(std::round(1.0 / hz * 1e9)));
        } else {
            interval_duration = std::chrono::nanoseconds { 0 };
        }
    };

    /**
     * @brief 使用给定配置初始化本地视频源并设置帧率间隔。
     *
     * 根据 _config.location 验证并打开视频源；若源提供的帧率大于零则使用之，否则使用 30 FPS 作为默认值。
     * 当 _config.frame_rate 大于零时，将其作为目标帧率覆盖源帧率；然后更新内部帧率间隔和最后读取时间点。
     *
     * @param _config 配置项，期望包含视频文件路径或设备标识（`location`）以及可选的 `frame_rate`（>0 表示覆盖目标帧率）。
     * @return std::expected<void, std::string> 成功时返回空的 expected；失败时返回包含错误描述的 unexpected，例如路径不存在或无法构造 VideoCapture。
     */
    auto configure(Config const& _config) -> std::expected<void, std::string> {
        if (_config.location.empty() || !std::filesystem::exists(_config.location)) {
            return std::unexpected { "Local video is not found or location is empty" };
        }

        config = _config;

        try {
            capturer.emplace(config.location);
        } catch (std::exception const& e) {
            return std::unexpected { "Failed to construct VideoCapture: " + std::string(e.what()) };
        } catch (...) {
            return std::unexpected { "Failed to construct VideoCapture due to an unknown error." };
        }

        double source_fps = capturer->get(cv::CAP_PROP_FPS);
        double target_fps = source_fps > 0 ? source_fps : 30.0;

        if (config.frame_rate > 0) {
            target_fps = config.frame_rate;
        }

        set_framerate_interval(target_fps);

        last_read_time = Clock::now();

        return {};
    }

    /**
 * @brief 使用当前已保存的配置建立或重新建立本地视频连接。
 *
 * 尝试基于内部保存的 Config 初始化或重新初始化视频捕获器并更新状态。
 *
 * @return 空值表示操作成功；错误时返回包含描述性错误信息的字符串。
 */
auto connect() -> std::expected<void, std::string> { return configure(config); }

    /**
 * @brief 判断本地视频捕获器当前是否已打开并可用。
 *
 * @return `true` 如果捕获器存在且已打开，`false` 否则。
 */
auto connected() const noexcept -> bool { return capturer.has_value() && capturer->isOpened(); }

    /**
     * @brief 断开并清理本地视频源的内部资源。
     *
     * 重置内部的 VideoCapture（如果存在）并将帧间隔重置为零，确保实现回到未连接状态。
     */
    auto disconnect() noexcept -> void {
        if (capturer.has_value()) {
            capturer.reset();
        }
        interval_duration = std::chrono::nanoseconds { 0 };
    }

    /**
     * @brief 从本地视频源读取下一帧并按配置的帧间隔返回封装的 Image。
     *
     * 等待直到与上次读取时间及帧率间隔匹配（或根据配置允许跳帧），从摄取器读取图像，
     * 在到达文件末尾时根据配置决定循环播放或返回错误，并在成功读取时为 Image 设置像素矩阵和时间戳。
     *
     * @returns std::expected<std::unique_ptr<Image>, std::string>
     * - 成功时包含 `std::unique_ptr<Image>`：其包含读取到的图像矩阵并已设置时间戳（基于内部时钟）。
     * - 失败时包含错误描述字符串，例如摄取器未打开、到达文件末尾、循环重置失败或读取到空帧。
     */
    auto wait_image() noexcept -> std::expected<std::unique_ptr<Image>, std::string> {
        if (!capturer.has_value() || !capturer->isOpened()) {
            return std::unexpected { "Video stream is not opened." };
        }

        const auto time_before_read        = Clock::now();
        const auto next_read_time_expected = last_read_time + interval_duration;
        auto wait_duration                 = next_read_time_expected - time_before_read;

        if (wait_duration.count() > 0) {
            std::this_thread::sleep_for(wait_duration);
            last_read_time = next_read_time_expected;
        } else {
            last_read_time = config.allow_skipping ? Clock::now() : next_read_time_expected;
        }

        auto frame = cv::Mat {};
        auto image = std::make_unique<Image>();
        if (!capturer->read(frame)) {
            if (config.loop_play) {
                if (capturer->set(cv::CAP_PROP_POS_FRAMES, 0) && capturer->read(frame)) {
                    last_read_time = Clock::now();
                } else {
                    return std::unexpected { "End of file reached and failed to "
                                             "loop/reset." };
                }
            } else {
                return std::unexpected { "End of file reached." };
            }
        }

        if (frame.empty()) {
            return std::unexpected { "Read frame is empty, possibly due to IO error." };
        }
        image->details().set_mat(frame);
        image->set_timestamp(last_read_time);

        return image;
    };
};

/**
 * @brief 使用给定配置初始化或更新本地视频源并打开捕获设备。
 *
 * 使用 config 中的参数（例如视频源路径和可选帧率）配置内部状态并尝试打开底层视频捕获器。
 *
 * @param config 包含视频源位置、目标帧率及回放选项的配置对象，用于初始化或更新本地视频捕获设置。
 * @return `void` 表示配置并打开捕获器成功；出错时返回包含错误描述的字符串。
 */
auto LocalVideo::configure(Config const& config) -> std::expected<void, std::string> {
    return pimpl->configure(config);
}
/**
 * @brief 等待并获取下一帧视频图像。
 *
 * 等待到下一个应读取的帧（基于配置的帧率与跳帧/循环策略），并返回封装的图像或描述性错误信息。
 *
 * @returns std::expected 包含 `std::unique_ptr<Image>` 表示成功时的图像；失败时包含描述性错误字符串，例如捕获设备未打开、读取失败、已到文件末尾或读取到空帧。
 */
auto LocalVideo::wait_image() noexcept -> std::expected<std::unique_ptr<Image>, std::string> {
    return pimpl->wait_image();
}

/**
 * @brief 尝试根据当前配置连接并初始化本地视频源。
 *
 * 初始化并打开内部视频捕获设备或文件，以便后续读取帧。若已经配置过有效的资源，则会复用该配置进行连接尝试。
 *
 * @returns std::expected<void, std::string> 成功时为空；失败时包含描述性错误消息（例如路径无效或打开设备失败）。
 */
auto LocalVideo::connect() noexcept -> std::expected<void, std::string> { return pimpl->connect(); }

/**
 * @brief 检查本地视频捕获器是否已成功打开并可用。
 *
 * @return `true` 如果底层视频捕获器已存在且已打开，`false` 否则。
 */
auto LocalVideo::connected() const noexcept -> bool { return pimpl->connected(); }

/**
 * @brief 关闭并释放本地视频源，重置帧率间隔。
 *
 * 释放内部视频捕获资源（如果有），并将内部帧间隔重置为零，恢复未连接状态。
 */
auto LocalVideo::disconnect() noexcept -> void { return pimpl->disconnect(); }

/**
     * @brief 构造一个 LocalVideo 实例并初始化内部实现（Pimpl）。
     *
     * 初始化并持有 Impl 的唯一所有权，用于封装本地视频处理的内部状态和行为。
     */
    LocalVideo::LocalVideo() noexcept
    : pimpl { std::make_unique<Impl>() } { }

LocalVideo::~LocalVideo() noexcept = default;