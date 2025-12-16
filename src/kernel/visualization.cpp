#include "visualization.hpp"

#include <fstream>

#include "module/debug/visualization/armor_visualizer.hpp"
#include "module/debug/visualization/stream_session.hpp"
#include "utility/image/image.details.hpp"
#include "utility/logging/printer.hpp"
#include "utility/serializable.hpp"

using namespace rmcs::kernel;
using namespace rmcs::util;

constexpr std::array kVideoTypes {
    "RTP_JEPG",
    "RTP_H264",
};

struct Visualization::Impl {
    using SessionConfig = debug::StreamSession::Config;
    using NormalResult  = std::expected<void, std::string>;

    struct Config : util::Serializable {

        util::integer_t framerate = 80;

        util::string_t monitor_host = "localhost";
        util::string_t monitor_port = "5000";

        util::string_t stream_type = "RTP_JEPG";

        static constexpr auto metas = std::tuple {
            &Config::framerate,
            "framerate",
            &Config::monitor_host,
            "monitor_host",
            &Config::monitor_port,
            "monitor_port",
            &Config::stream_type,
            "stream_type",
        };
    };

    Printer log { "visualization" };

    std::unique_ptr<debug::StreamSession> session;
    SessionConfig session_config;

    bool is_initialized  = false;
    bool size_determined = false;

    std::unique_ptr<debug::ArmorVisualizer> armor_visualizer;

    /**
     * @brief 构造 Visualization::Impl 的实例并初始化内部资源。
     *
     * 在构造时为视频流会话和装甲可视化器分配并构造默认实例，
     * 以便后续初始化和使用。
     */
    Impl() noexcept {
        session          = std::make_unique<debug::StreamSession>();
        armor_visualizer = std::make_unique<debug::ArmorVisualizer>();
    }

    /**
     * @brief 使用给定的 YAML 配置和 ROS 节点初始化可视化子系统。
     *
     * 解析并应用配置（包括帧率、监视主机/端口和视频流类型），配置内部的流会话参数，并使用提供的 ROS 节点初始化装甲可视化器。
     *
     * @param yaml 包含可配置项的 YAML 节点（期望字段：framerate、monitor_host、monitor_port、stream_type）。
     * @param visual_node 用于初始化 ArmorVisualizer 的 ROS 节点。
     * @return std::expected<void, std::string> 空值表示成功；包含错误描述的字符串表示初始化失败（例如配置序列化失败或未知的视频类型）。
     */
    auto initialize(const YAML::Node& yaml, RclcppNode& visual_node) noexcept -> NormalResult {
        auto config = Config {};
        auto result = config.serialize(yaml);
        if (!result.has_value()) {
            return std::unexpected { result.error() };
        }

        session_config.target.host = config.monitor_host;
        session_config.target.port = config.monitor_port;
        session_config.format.hz   = static_cast<int>(config.framerate);

        if (config.stream_type == kVideoTypes[0]) {
            session_config.type = debug::StreamType::RTP_JEPG;
        } else if (config.stream_type == kVideoTypes[1]) {
            session_config.type = debug::StreamType::RTP_H264;
        } else {
            return std::unexpected { "Unknown video type: " + config.stream_type };
        }

        armor_visualizer->initialize(visual_node);

        is_initialized = true;
        return {};
    }

    auto initialized() const noexcept { return is_initialized; }

    /**
     * @brief 将给定图像的像素矩阵发送到已配置的视频会话进行实时推流。
     *
     * 如果尚未确定帧尺寸，函数会使用图像矩阵的宽高更新会话配置、打开会话并将会话描述（SDP）写入 /tmp/auto_aim.sdp。函数还会在会话未打开或发生任何初始化/写入/打开错误时返回失败。
     *
     * @param image 要推送的图像，其内部包含用于推流的 OpenCV 矩阵（mat）。
     * @return true 如果帧已成功推送到会话，false 如果未初始化、会话打开失败、SDP 写入失败、会话未打开或推送失败。
     */
    auto send_image(const Image& image) noexcept -> bool {
        if (!is_initialized) return false;

        const auto& mat = image.details().mat;

        if (!size_determined) {
            session_config.format.w = mat.cols;
            session_config.format.h = mat.rows;

            { // open session
                auto ret = session->open(session_config);
                if (!ret) {
                    log.error("Failed to open visualization session");
                    log.error("  e: {}", ret.error());
                    return false;
                }
                log.info("Visualization session is opened");
            }
            { // write SDP
                auto ret = session->session_description_protocol();
                if (!ret) {
                    log.error("Failed to get description protocol");
                    log.error("  e: {}", ret.error());
                    return false;
                }

                const auto output_location = "/tmp/auto_aim.sdp";
                if (auto ofstream = std::ofstream { output_location }) {
                    ofstream << ret.value();
                    log.info("Sdp has been written to: {}", output_location);
                } else {
                    log.error("Failed to write sdp: {}", output_location);
                    return false;
                }
            }
            size_determined = true;
        }
        if (!session->opened()) return false;

        return session->push_frame(mat);
    }
    /**
     * 使用内部 ArmorVisualizer 对一组三维装甲目标进行可视化并发送到可视化管道。
     *
     * @param armors 要可视化的 Armor3D 对象序列。
     * @return `true` 表示可视化并发送成功，`false` 表示未初始化或可视化失败。 
     */
    auto visualize_armors(std::span<Armor3D> const& armors) const -> bool {
        if (!is_initialized) return false;
        return armor_visualizer->visualize(armors);
    }
};

/**
 * @brief 使用 YAML 配置和可视化节点初始化 Visualization 实例。
 *
 * 从给定的 YAML 节点读取可配置项（例如帧率、监视主机/端口和流媒体类型），并使用提供的 RclcppNode 初始化用于渲染的可视化子系统（例如 ArmorVisualizer）。
 *
 * @param yaml 包含 Visualization 配置的 YAML 节点（应含 framerate、monitor_host、monitor_port、stream_type 等条目）。
 * @param visual_node 用于初始化可视化/渲染组件的 ROS 节点实例。
 * @return std::expected<void, std::string> 空值表示初始化成功；若初始化失败则返回包含错误描述的 std::string。
 */
auto Visualization::initialize(const YAML::Node& yaml, RclcppNode& visual_node) noexcept
    -> std::expected<void, std::string> {
    return pimpl->initialize(yaml, visual_node);
}

/**
 * @brief 检查可视化子系统是否已完成初始化。
 *
 * @return `true` 如果已初始化，`false` 否则。
 */
auto Visualization::initialized() const noexcept -> bool { return pimpl->initialized(); }

/**
 * @brief 将一帧图像发送到已配置的可视化流。
 *
 * 该方法将图像转发给内部实现以进行流推送；在未初始化或发生错误时返回 `false`。
 *
 * @param image 要发送的图像帧。
 * @return `true` 如果图像成功推送或入队，`false` 否则。
 */
auto Visualization::send_image(const Image& image) noexcept -> bool {
    return pimpl->send_image(image);
}

/**
 * @brief 将一组装甲三维数据提交给可视化模块进行渲染。
 *
 * 将提供的 Armor3D 列表传递给内部的可视化器以绘制并展示这些装甲对象。
 *
 * @param armors 要渲染的一组装甲对象（Armor3D 的视图）。
 * @return `true` 表示渲染并提交成功，`false` 表示失败或可视化尚未初始化。
 */
auto Visualization::visualize_armors(std::span<Armor3D> const& armors) const -> bool {
    return pimpl->visualize_armors(armors);
}

/**
     * @brief 创建一个 Visualization 对象并构建其内部实现（PIMPL）。
     *
     * 为该实例分配并初始化内部 Impl 对象以隐藏具体实现细节。
     */
    Visualization::Visualization() noexcept
    : pimpl { std::make_unique<Impl>() } { }

Visualization::~Visualization() noexcept = default;