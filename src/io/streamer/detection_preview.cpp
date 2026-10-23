#include <algorithm>
#include <array>
#include <atomic>
#include <charconv>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <format>
#include <fstream>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>

#include <fcntl.h>
#include <poll.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/camera_frame.hpp>
#include <rmcs_utility/pooled_shared_factory.hpp>

#include "kernel/detector.hpp"
#include "utility/image/drawable.hpp"
#include "utility/rclcpp/configuration.hpp"

namespace rmcs {
namespace {

using Clock = std::chrono::steady_clock;
using Frame = rmcs_msgs::CameraFrame;
using FramePtr = std::shared_ptr<const Frame>;
using namespace std::chrono_literals;
namespace fs = std::filesystem;

enum class PreviewStatus { Idle, Starting, Running, Error, Unavailable };

auto status_name(PreviewStatus status) -> std::string_view {
    switch (status) {
    case PreviewStatus::Idle: return "idle";
    case PreviewStatus::Starting: return "starting";
    case PreviewStatus::Running: return "running";
    case PreviewStatus::Error: return "error";
    case PreviewStatus::Unavailable: return "unavailable";
    }
    return "error";
}

struct RequestResult {
    std::string id;
    bool ok = false;
    std::string error;
};

struct PreviewState {
    bool available = false;
    bool enabled = false;
    PreviewStatus status = PreviewStatus::Unavailable;
    std::string error = "Waiting for the tunable capturer";
    RequestResult last_request;
};

auto unix_ms() -> std::int64_t {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

auto json_string(std::string_view value) -> std::string {
    std::string result = "\"";
    for (const unsigned char ch : value) {
        if (ch == '"' || ch == '\\') {
            result += '\\';
            result += static_cast<char>(ch);
        } else if (ch < 0x20) {
            result += std::format("\\u{:04x}", ch);
        } else {
            result += static_cast<char>(ch);
        }
    }
    return result + '"';
}

auto valid_component_name(std::string_view name) -> bool {
    const auto valid_start = [](unsigned char ch) {
        return (ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') || ch == '_';
    };
    return !name.empty() && valid_start(name.front())
        && std::ranges::all_of(name, [&](unsigned char ch) {
               return valid_start(ch) || (ch >= '0' && ch <= '9');
           });
}

// This IPC is independent of camera configuration, so loading/inference never stalls its heartbeat.
class PreviewControl {
public:
    explicit PreviewControl(std::string component_name)
        : component_name_(std::move(component_name))
        , directory_(fs::path("/tmp/autoaim/detection") / component_name_) {
        if (!valid_component_name(component_name_)) {
            throw std::runtime_error("Invalid detection capturer_component");
        }
        try {
            fs::create_directories(directory_);
            lock_fd_ = ::open((directory_ / ".lock").c_str(), O_CREAT | O_RDWR | O_CLOEXEC, 0644);
            if (lock_fd_ < 0 || ::flock(lock_fd_, LOCK_EX | LOCK_NB) != 0) {
                throw std::runtime_error("Detection control is already running or cannot be locked");
            }
            owns_lock_ = true;
            const auto commands = directory_ / "commands";
            struct stat info {};
            if (::lstat(commands.c_str(), &info) == 0) {
                if (!S_ISFIFO(info.st_mode)) {
                    throw std::runtime_error("Detection commands path is not a FIFO");
                }
                fs::remove(commands);
            }
            if (::mkfifo(commands.c_str(), 0660) != 0) {
                throw std::runtime_error("Cannot create detection commands FIFO");
            }
            owns_fifo_ = true;
            fifo_fd_ = ::open(commands.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
            if (fifo_fd_ < 0) throw std::runtime_error("Cannot open detection commands FIFO");
        } catch (...) {
            cleanup();
            throw;
        }
    }

    ~PreviewControl() { cleanup(); }
    PreviewControl(const PreviewControl&) = delete;
    auto operator=(const PreviewControl&) -> PreviewControl& = delete;

    auto publish(const PreviewState& state, bool force = false) -> void {
        const auto now = Clock::now();
        if (!force && now < next_publish_) return;
        next_publish_ = now + 100ms;
        const auto json = std::format(
            "{{\"pid\":{},\"updated_at\":{},\"capturer_component\":{},"
            "\"available\":{},\"enabled\":{},\"status\":{},\"error\":{},"
            "\"last_request\":{{\"id\":{},\"ok\":{},\"error\":{}}}}}\n",
            ::getpid(), static_cast<double>(unix_ms()) / 1000.0, json_string(component_name_),
            state.available, state.enabled, json_string(status_name(state.status)),
            json_string(state.error), json_string(state.last_request.id), state.last_request.ok,
            json_string(state.last_request.error));
        const auto temporary = directory_ / "state.json.tmp";
        {
            auto output = std::ofstream(temporary, std::ios::trunc);
            output << json;
            output.close();
            if (!output) throw std::runtime_error("Cannot write detection state");
        }
        fs::rename(temporary, directory_ / "state.json");
    }

    template <typename Callback>
    auto drain(Callback&& callback) -> void {
        char buffer[512];
        for (int reads = 0; reads < 8; ++reads) {
            const auto count = ::read(fifo_fd_, buffer, sizeof(buffer));
            if (count <= 0) break;
            for (std::ptrdiff_t i = 0; i < count; ++i) {
                if (buffer[i] == '\n') {
                    if (!discard_line_ && !pending_.empty()) callback(pending_);
                    pending_.clear();
                    discard_line_ = false;
                } else if (!discard_line_) {
                    if (pending_.size() >= 256) {
                        pending_.clear();
                        discard_line_ = true;
                    } else {
                        pending_ += buffer[i];
                    }
                }
            }
        }
    }

    auto wait() const -> void {
        auto descriptor = pollfd { fifo_fd_, POLLIN, 0 };
        (void)::poll(&descriptor, 1, 100);
    }

private:
    auto cleanup() noexcept -> void {
        if (fifo_fd_ >= 0) ::close(fifo_fd_);
        if (owns_lock_) {
            std::error_code ignored;
            if (owns_fifo_) fs::remove(directory_ / "commands", ignored);
            fs::remove(directory_ / "state.json", ignored);
            fs::remove(directory_ / "state.json.tmp", ignored);
        }
        // Preserve the inode so a replacement instance cannot bypass a held lock.
        if (lock_fd_ >= 0) ::close(lock_fd_);
        fifo_fd_ = lock_fd_ = -1;
        owns_lock_ = false;
    }

    const std::string component_name_;
    const fs::path directory_;
    int lock_fd_ = -1;
    int fifo_fd_ = -1;
    bool owns_lock_ = false;
    bool owns_fifo_ = false;
    bool discard_line_ = false;
    std::string pending_;
    Clock::time_point next_publish_ {};
};

auto initialize_detector() -> std::unique_ptr<kernel::Detector> {
    const auto configs = YAML::Clone(util::configs());
    auto detector_config = YAML::Clone(configs["detector"]);
    const auto model_path = fs::path(detector_config["model_location"].as<std::string>());
    detector_config["model_location"] = model_path.is_absolute() ? model_path.string()
        : (fs::path(util::Parameters::share_location()) / model_path).string();
    auto detector = std::make_unique<kernel::Detector>();
    const auto initialized = detector->initialize(detector_config);
    if (!initialized) throw std::runtime_error(initialized.error());
    detector->update_camera(configs["camera_matrix"].as<std::array<double, 9>>());
    detector->update_camera(configs["distort_coeff"].as<std::array<double, 5>>());
    return detector;
}

auto draw_result(Canvas& canvas, const kernel::Detector::Result& result,
    std::string_view rune_color, const cv::Scalar& color) -> void {
    for (const auto& armor : result.armors) canvas.draw(armor);
    for (const auto& lightbar : result.lightbars) canvas.draw(lightbar);
    for (const auto& area : result.areas) canvas.draw(area);
    if (result.green_light) {
        canvas.draw(*result.green_light);
        canvas.draw(Canvas::Text { "GREEN LIGHT", result.green_light->tl(), kGreen });
    }
    for (const auto& icon : result.icons) {
        const auto center = icon.center.make<cv::Point2i>();
        canvas.draw(Canvas::Point { center, 5, color });
        canvas.draw(Canvas::Text { std::format("R {}: {:.3f}", rune_color, icon.score), center, color });
    }
    for (const auto& bullseye : result.bullseyes) {
        if (!bullseye.active) {
            for (const auto& corner : bullseye.corners) {
                canvas.draw(Canvas::Point { corner.make<cv::Point2i>(), 3, color });
            }
        }
        const auto center = bullseye.center.make<cv::Point2i>();
        canvas.draw(Canvas::Point { center, 5, color });
        canvas.draw(Canvas::Text {
            std::format("B {} {}: {:.3f}", rune_color, bullseye.active ? "ACTIVE" : "INACTIVE",
                bullseye.score),
            center, color });
    }
}

} // namespace

class AutoAimDetectionPreviewComponent final : public rmcs_executor::Component, public rclcpp::Node {
public:
    AutoAimDetectionPreviewComponent()
        : Node(get_component_name(),
              rclcpp::NodeOptions {}.automatically_declare_parameters_from_overrides(true))
        , capturer_component_(get_parameter_or<std::string>("capturer_component", "auto_aim_capturer")) {
        const auto input_topic = get_parameter_or<std::string>("frame_topic", "/gimbal/auto_aim/camera_frame");
        const auto output_topic = get_parameter_or<std::string>("output_frame_topic", "/gimbal/auto_aim/detection_preview_frame");
        if (input_topic.empty() || output_topic.empty() || input_topic == output_topic) {
            throw std::runtime_error("Detection input/output frame topics must be nonempty and distinct");
        }
        control_ = std::make_unique<PreviewControl>(capturer_component_);
        register_input(input_topic, frame_input_);
        register_output(output_topic, frame_output_);
        register_input(std::format("/{}/tunable_capture_available", capturer_component_), available_input_, false);
        control_->publish(state_, true);
    }

    ~AutoAimDetectionPreviewComponent() override {
        stop_requested_.store(true, std::memory_order_relaxed);
        {
            const auto lock = std::lock_guard(mutex_);
            invalidate_locked();
        }
        changed_.notify_all();
        if (service_thread_.joinable()) service_thread_.join();
        if (inference_thread_.joinable()) inference_thread_.join();
    }

    auto before_updating() -> void override {
        update();
        inference_thread_ = std::thread([this] { inference_main(); });
        try {
            service_thread_ = std::thread([this] { service_main(); });
        } catch (...) {
            stop_requested_.store(true, std::memory_order_relaxed);
            changed_.notify_all();
            inference_thread_.join();
            throw;
        }
        RCLCPP_INFO(get_logger(), "Detection preview control: /tmp/autoaim/detection/%s/",
            capturer_component_.c_str());
    }

    auto update() -> void override {
        // Normal component inputs belong to the executor thread, never to the event/worker threads.
        const auto available = available_input_.ready() && *available_input_;
        if (available_.exchange(available, std::memory_order_acq_rel) == available) return;
        {
            const auto lock = std::lock_guard(mutex_);
            state_.available = available;
            invalidate_locked();
            state_.status = available ? PreviewStatus::Idle : PreviewStatus::Unavailable;
            state_.error = available ? "" : "The tunable capturer is unavailable";
        }
        changed_.notify_all();
    }

private:
    auto invalidate_locked() -> void {
        state_.enabled = false;
        ++generation_;
        latest_frame_.reset();
    }

    auto frame_callback(const FramePtr& frame) -> void {
        if (!frame || stop_requested_.load(std::memory_order_relaxed)) return;
        const auto lock = std::lock_guard(mutex_);
        if (!available_.load(std::memory_order_acquire) || !state_.enabled
            || state_.status != PreviewStatus::Running) {
            // Keep video and recording independent of model loading and detection failures.
            frame_output_.emit(frame);
            return;
        }
        latest_frame_ = frame;
        changed_.notify_one();
    }

    auto state_snapshot() -> PreviewState {
        const auto lock = std::lock_guard(mutex_);
        return state_;
    }

    auto handle_command(const std::string& line) -> void {
        std::istringstream input(line);
        std::string id, enabled, expires_text, extra;
        if (!(input >> id >> enabled >> expires_text) || (input >> extra) || id.size() > 64
            || !std::ranges::all_of(id, [](unsigned char ch) {
                   return (ch >= '0' && ch <= '9') || (ch >= 'a' && ch <= 'f')
                       || (ch >= 'A' && ch <= 'F');
               })) return;
        std::int64_t expires = 0;
        const auto parsed = std::from_chars(expires_text.data(), expires_text.data() + expires_text.size(), expires);
        const auto lock = std::lock_guard(mutex_);
        state_.last_request = { .id = id, .ok = false, .error = {} };
        if (parsed.ec != std::errc {} || parsed.ptr != expires_text.data() + expires_text.size()
            || (enabled != "0" && enabled != "1")) {
            state_.last_request.error = "Invalid detection command";
        } else if (expires <= unix_ms()) {
            state_.last_request.error = "Detection request expired";
        } else if (enabled == "1" && (!available_.load(std::memory_order_acquire) || !state_.available)) {
            state_.last_request.error = "The tunable capturer is unavailable";
        } else {
            if (enabled == "0") {
                invalidate_locked();
                state_.status = state_.available ? PreviewStatus::Idle : PreviewStatus::Unavailable;
                state_.error = state_.available ? "" : "The tunable capturer is unavailable";
            } else if (!state_.enabled) {
                ++generation_;
                latest_frame_.reset();
                state_.enabled = true;
                state_.status = model_ready_ ? PreviewStatus::Running : PreviewStatus::Starting;
                state_.error.clear();
            }
            // Start acknowledges the request; loading completes asynchronously in the inference thread.
            state_.last_request.ok = true;
            changed_.notify_one();
        }
    }

    auto service_main() -> void {
        try {
            while (!stop_requested_.load(std::memory_order_relaxed)) {
                control_->drain([this](const std::string& line) {
                    handle_command(line);
                    control_->publish(state_snapshot(), true);
                });
                control_->publish(state_snapshot());
                control_->wait();
            }
        } catch (const std::exception& error) {
            fail_detection(error.what());
            RCLCPP_ERROR(get_logger(), "Detection control stopped: %s", error.what());
        } catch (...) {
            fail_detection("Unknown detection control error");
            RCLCPP_ERROR(get_logger(), "Detection control stopped with an unknown error");
        }
    }

    auto fail_detection(std::string error) -> void {
        const auto lock = std::lock_guard(mutex_);
        invalidate_locked();
        state_.status = state_.available ? PreviewStatus::Error : PreviewStatus::Unavailable;
        state_.error = std::move(error);
    }

    auto fail_generation(std::uint64_t generation, const std::string& error) -> void {
        const auto lock = std::lock_guard(mutex_);
        if (generation != generation_ || !state_.enabled) return;
        invalidate_locked();
        state_.status = state_.available ? PreviewStatus::Error : PreviewStatus::Unavailable;
        state_.error = error;
        RCLCPP_WARN(get_logger(), "Detection preview disabled: %s", error.c_str());
    }

    auto generation_active(std::uint64_t generation) -> bool {
        const auto lock = std::lock_guard(mutex_);
        return !stop_requested_.load(std::memory_order_relaxed) && generation == generation_
            && state_.enabled && available_.load(std::memory_order_acquire);
    }

    auto inference_main() -> void {
        std::unique_ptr<kernel::Detector> detector;
        while (!stop_requested_.load(std::memory_order_relaxed)) {
            FramePtr source;
            std::uint64_t generation = 0;
            bool initialize = false;
            {
                auto lock = std::unique_lock(mutex_);
                changed_.wait(lock, [this] {
                    return stop_requested_.load(std::memory_order_relaxed)
                        || (state_.enabled && available_.load(std::memory_order_acquire)
                            && (!model_ready_ || latest_frame_));
                });
                if (stop_requested_.load(std::memory_order_relaxed)) return;
                generation = generation_;
                initialize = !model_ready_;
                if (!initialize) source = std::move(latest_frame_);
            }
            try {
                if (!generation_active(generation)) continue;
                if (initialize) {
                    detector = initialize_detector();
                    const auto lock = std::lock_guard(mutex_);
                    model_ready_ = true;
                    // A stop/restart during loading can reuse this model for the current request.
                    if (state_.enabled && available_.load(std::memory_order_acquire)) {
                        state_.status = PreviewStatus::Running;
                    }
                    continue;
                }
                if (!source) continue;
                auto output = frame_factory_.try_make(*source);
                if (output) {
                    auto image = cv::Mat(static_cast<int>(Frame::kHeight), static_cast<int>(Frame::kWidth),
                        CV_8UC3, reinterpret_cast<unsigned char*>(output->data.data()));
                    if (!generation_active(generation)) continue;
                    detector->update_detect_rune(false);
                    const auto armor = detector->detect(image);
                    if (!generation_active(generation)) continue;
                    detector->update_detect_rune(true);
                    // Detector intentionally reverses this setting: BLUE selects red rune pixels.
                    detector->update_detect_color(CampColor::BLUE);
                    const auto red_rune = detector->detect(image);
                    if (!generation_active(generation)) continue;
                    detector->update_detect_color(CampColor::RED);
                    const auto blue_rune = detector->detect(image);
                    if (!generation_active(generation)) continue;
                    // All three passes consume unannotated pixels; draw only after they finish.
                    auto canvas = Canvas { image };
                    draw_result(canvas, armor, "", kGreen);
                    draw_result(canvas, red_rune, "RED", kRed);
                    draw_result(canvas, blue_rune, "BLUE", kCyan);
                }
                const auto lock = std::lock_guard(mutex_);
                // Serialize the final generation check and emission with stop/unavailable acknowledgements.
                if (!stop_requested_.load(std::memory_order_relaxed) && generation == generation_
                    && state_.enabled && available_.load(std::memory_order_acquire)) {
                    if (output) frame_output_.emit(output);
                    else frame_output_.emit(source);
                }
            } catch (const std::exception& error) {
                fail_generation(generation, error.what());
            } catch (...) {
                fail_generation(generation, "Unknown detection preview error");
            }
        }
    }

    const std::string capturer_component_;
    std::unique_ptr<PreviewControl> control_;
    std::mutex mutex_;
    std::condition_variable changed_;
    PreviewState state_;
    bool model_ready_ = false;
    std::uint64_t generation_ = 0;
    FramePtr latest_frame_;
    std::atomic<bool> available_ { false };
    std::atomic<bool> stop_requested_ { false };
    rmcs_utility::PooledSharedFactory<Frame> frame_factory_ { 8 };
    InputInterface<bool> available_input_;
    EventOutputInterface<FramePtr> frame_output_;
    EventInputInterface<FramePtr> frame_input_ {
        [this](const FramePtr& frame) { frame_callback(frame); },
    };
    std::thread inference_thread_;
    std::thread service_thread_;
};

} // namespace rmcs

PLUGINLIB_EXPORT_CLASS(rmcs::AutoAimDetectionPreviewComponent, rmcs_executor::Component)
