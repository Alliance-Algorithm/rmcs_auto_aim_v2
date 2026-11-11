#include <array>
#include <atomic>
#include <charconv>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <filesystem>
#include <format>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/camera_frame.hpp>
#include <rmcs_utility/pooled_shared_factory.hpp>

#include "kernel/detector.hpp"
#include "utility/control_channel.hpp"
#include "utility/image/drawable.hpp"
#include "utility/rclcpp/configuration.hpp"

namespace rmcs {

// Annotates preview frames on demand; frames pass through untouched while detection is off.
class AutoAimDetectionPreviewComponent final : public rmcs_executor::Component,
                                               public rclcpp::Node {
public:
    AutoAimDetectionPreviewComponent()
        : Node(get_component_name(),
              rclcpp::NodeOptions { }.automatically_declare_parameters_from_overrides(true)) {
        const auto input_topic =
            get_parameter_or<std::string>("frame_topic", "/gimbal/auto_aim/camera_frame");
        const auto output_topic = get_parameter_or<std::string>(
            "output_frame_topic", "/gimbal/auto_aim/detection_preview_frame");
        if (input_topic.empty() || output_topic.empty() || input_topic == output_topic) {
            throw std::runtime_error {
                "Detection input/output frame topics must be nonempty and distinct",
            };
        }

        register_input(input_topic, frame_input_);
        register_output(output_topic, frame_output_);
        register_input(std::format("/{}/tunable_capture_available", capturer_component_),
            available_input_, false);

        publish_state(true);
    }

    ~AutoAimDetectionPreviewComponent() override {
        stop_requested_.store(true, std::memory_order::relaxed);
        {
            const auto lock = std::lock_guard { mutex_ };
            invalidate_locked();
        }
        work_changed_.notify_all();
        if (service_thread_.joinable()) service_thread_.join();
        if (inference_thread_.joinable()) inference_thread_.join();
    }

    void before_updating() override {
        update();
        inference_thread_ = std::thread { [this] { inference_main(); } };
        try {
            service_thread_ = std::thread { [this] { service_main(); } };
        } catch (...) {
            stop_requested_.store(true, std::memory_order::relaxed);
            work_changed_.notify_all();
            inference_thread_.join();
            throw;
        }
        RCLCPP_INFO(
            logger_, "Detection preview control: %s/", control_channel_.directory().c_str());
    }

    void update() override {
        // Normal component inputs belong to the executor thread, never to the event/worker threads.
        const auto available = available_input_.ready() && *available_input_;
        if (available_.exchange(available, std::memory_order::acq_rel) == available) return;
        {
            const auto lock  = std::lock_guard { mutex_ };
            state_.available = available;
            invalidate_locked();
            reset_status_locked();
        }
        work_changed_.notify_all();
    }

private:
    enum class Status { Idle, Starting, Running, Error, Unavailable };

    struct RequestResult {
        std::string id;
        bool ok = false;
        std::string error;
    };

    struct State {
        bool available    = false;
        bool enabled      = false;
        Status status     = Status::Unavailable;
        std::string error = "Waiting for the tunable capturer";
        RequestResult last_request;
    };

    void handle_frame(const std::shared_ptr<const rmcs_msgs::CameraFrame>& frame) {
        if (!frame || stop_requested_.load(std::memory_order::relaxed)) return;
        const auto lock = std::lock_guard { mutex_ };
        if (!available_.load(std::memory_order::acquire) || !state_.enabled
            || state_.status != Status::Running) {
            // Keep video and recording independent of model loading and detection failures.
            frame_output_.emit(frame);
            return;
        }
        latest_frame_ = frame;
        work_changed_.notify_one();
    }

    void service_main() {
        try {
            while (!stop_requested_.load(std::memory_order::relaxed)) {
                control_channel_.drain_commands([this](const std::string& line) {
                    handle_command(line);
                    publish_state(true);
                });
                publish_state();
                control_channel_.wait_for_command(std::chrono::milliseconds { 100 });
            }
        } catch (const std::exception& error) {
            fail(error.what());
            RCLCPP_ERROR(logger_, "Detection control stopped: %s", error.what());
        } catch (...) {
            fail("Unknown detection control error");
            RCLCPP_ERROR(logger_, "Detection control stopped with an unknown error");
        }
    }

    void handle_command(const std::string& line) {
        auto input = std::istringstream { line };
        std::string id, enabled, expires_text, extra;
        if (!(input >> id >> enabled >> expires_text) || (input >> extra)) return;
        if (!util::ControlChannel::valid_request_id(id)) return;

        const auto* const expires_end = expires_text.data() + expires_text.size();
        auto expires                  = std::int64_t { 0 };
        const auto parsed             = std::from_chars(expires_text.data(), expires_end, expires);

        const auto lock     = std::lock_guard { mutex_ };
        state_.last_request = RequestResult { .id = id, .ok = false, .error = { } };
        if (parsed.ec != std::errc { } || parsed.ptr != expires_end
            || (enabled != "0" && enabled != "1")) {
            state_.last_request.error = "Invalid detection command";
        } else if (expires <= util::unix_ms()) {
            state_.last_request.error = "Detection request expired";
        } else if (enabled == "1"
            && (!available_.load(std::memory_order::acquire) || !state_.available)) {
            state_.last_request.error = kCapturerUnavailable;
        } else {
            if (enabled == "0") {
                invalidate_locked();
                reset_status_locked();
            } else if (!state_.enabled) {
                ++generation_;
                latest_frame_.reset();
                state_.enabled = true;
                state_.status  = model_ready_ ? Status::Running : Status::Starting;
                state_.error.clear();
            }
            // Starting only acknowledges the request; the inference thread loads the model.
            state_.last_request.ok = true;
            work_changed_.notify_one();
        }
    }

    void publish_state(bool force = false) {
        const auto now = std::chrono::steady_clock::now();
        if (!force && now < next_publish_time_) return;
        next_publish_time_ = now + std::chrono::milliseconds { 100 };

        const auto state = state_snapshot();
        control_channel_.write_state(std::format(
            R"("capturer_component":{},"available":{},"enabled":{},"status":{},"error":{},)"
            R"("last_request":{{"id":{},"ok":{},"error":{}}})",
            util::json_string(capturer_component_), state.available, state.enabled,
            util::json_string(status_name(state.status)), util::json_string(state.error),
            util::json_string(state.last_request.id), state.last_request.ok,
            util::json_string(state.last_request.error)));
    }

    [[nodiscard]] auto state_snapshot() -> State {
        const auto lock = std::lock_guard { mutex_ };
        return state_;
    }

    void inference_main() {
        auto detector = std::unique_ptr<kernel::Detector> { };
        while (!stop_requested_.load(std::memory_order::relaxed)) {
            auto source     = std::shared_ptr<const rmcs_msgs::CameraFrame> { };
            auto generation = std::uint64_t { 0 };
            auto load_model = false;
            {
                auto lock = std::unique_lock { mutex_ };
                work_changed_.wait(lock, [this] {
                    return stop_requested_.load(std::memory_order::relaxed)
                        || (state_.enabled && available_.load(std::memory_order::acquire)
                            && (!model_ready_ || latest_frame_));
                });
                if (stop_requested_.load(std::memory_order::relaxed)) return;
                generation = generation_;
                load_model = !model_ready_;
                if (!load_model) source = std::move(latest_frame_);
            }

            try {
                if (!generation_active(generation)) continue;
                if (load_model) {
                    detector = make_detector();
                    mark_model_ready();
                } else if (source) {
                    emit_preview(*detector, source, generation);
                }
            } catch (const std::exception& error) {
                fail_generation(generation, error.what());
            } catch (...) {
                fail_generation(generation, "Unknown detection preview error");
            }
        }
    }

    void mark_model_ready() {
        const auto lock = std::lock_guard { mutex_ };
        model_ready_    = true;
        // A stop/restart during loading can reuse this model for the current request.
        if (state_.enabled && available_.load(std::memory_order::acquire)) {
            state_.status = Status::Running;
        }
    }

    void emit_preview(kernel::Detector& detector,
        const std::shared_ptr<const rmcs_msgs::CameraFrame>& source, std::uint64_t generation) {
        // Without a free pooled frame, forward the source unannotated instead of dropping it.
        auto output = frame_factory_.try_make(*source);
        if (output && !annotate(detector, *output, generation)) return;

        const auto lock = std::lock_guard { mutex_ };
        // Serialize the final generation check and emission with stop/unavailable acknowledgements.
        if (!generation_active_locked(generation)) return;
        if (output) {
            frame_output_.emit(output);
        } else {
            frame_output_.emit(source);
        }
    }

    [[nodiscard]] auto annotate(kernel::Detector& detector, rmcs_msgs::CameraFrame& frame,
        std::uint64_t generation) -> bool {
        auto image = cv::Mat { static_cast<int>(rmcs_msgs::CameraFrame::kHeight),
            static_cast<int>(rmcs_msgs::CameraFrame::kWidth), CV_8UC3,
            reinterpret_cast<unsigned char*>(frame.data.data()) };

        if (!generation_active(generation)) return false;
        detector.update_detect_rune(false);
        const auto armor = detector.detect(image);

        if (!generation_active(generation)) return false;
        detector.update_detect_rune(true);
        // Detector intentionally reverses this setting: BLUE selects red rune pixels.
        detector.update_detect_color(CampColor::BLUE);
        const auto red_rune = detector.detect(image);

        if (!generation_active(generation)) return false;
        detector.update_detect_color(CampColor::RED);
        const auto blue_rune = detector.detect(image);

        if (!generation_active(generation)) return false;
        // All three passes consume unannotated pixels; draw only after they finish.
        auto canvas = Canvas { image };
        draw_result(canvas, armor, "", kGreen);
        draw_result(canvas, red_rune, "RED", kRed);
        draw_result(canvas, blue_rune, "BLUE", kCyan);
        return true;
    }

    [[nodiscard]] auto generation_active(std::uint64_t generation) -> bool {
        const auto lock = std::lock_guard { mutex_ };
        return generation_active_locked(generation);
    }

    [[nodiscard]] auto generation_active_locked(std::uint64_t generation) const -> bool {
        return !stop_requested_.load(std::memory_order::relaxed) && generation == generation_
            && state_.enabled && available_.load(std::memory_order::acquire);
    }

    void fail(std::string error) {
        const auto lock = std::lock_guard { mutex_ };
        fail_locked(std::move(error));
    }

    void fail_generation(std::uint64_t generation, std::string error) {
        const auto lock = std::lock_guard { mutex_ };
        if (generation != generation_ || !state_.enabled) return;
        RCLCPP_WARN(logger_, "Detection preview disabled: %s", error.c_str());
        fail_locked(std::move(error));
    }

    void fail_locked(std::string error) {
        invalidate_locked();
        state_.status = state_.available ? Status::Error : Status::Unavailable;
        state_.error  = std::move(error);
    }

    void invalidate_locked() {
        state_.enabled = false;
        ++generation_;
        latest_frame_.reset();
    }

    void reset_status_locked() {
        state_.status = state_.available ? Status::Idle : Status::Unavailable;
        state_.error  = state_.available ? "" : kCapturerUnavailable;
    }

    [[nodiscard]] static auto status_name(Status status) -> std::string_view {
        switch (status) {
        case Status::Idle:
            return "idle";
        case Status::Starting:
            return "starting";
        case Status::Running:
            return "running";
        case Status::Error:
            return "error";
        case Status::Unavailable:
            return "unavailable";
        }
        return "error";
    }

    [[nodiscard]] static auto make_detector() -> std::unique_ptr<kernel::Detector> {
        const auto configs   = YAML::Clone(util::configs());
        auto detector_config = YAML::Clone(configs["detector"]);
        const auto model_path =
            std::filesystem::path { detector_config["model_location"].as<std::string>() };
        detector_config["model_location"] = model_path.is_absolute()
            ? model_path.string()
            : (std::filesystem::path { util::Parameters::share_location() } / model_path).string();

        auto detector = std::make_unique<kernel::Detector>();
        if (const auto result = detector->initialize(detector_config); !result) {
            throw std::runtime_error { result.error() };
        }
        detector->update_camera(configs["camera_matrix"].as<std::array<double, 9>>());
        detector->update_camera(configs["distort_coeff"].as<std::array<double, 5>>());
        return detector;
    }

    static void draw_result(Canvas& canvas, const kernel::Detector::Result& result,
        std::string_view rune_color, const cv::Scalar& color) {
        for (const auto& armor : result.armors)
            canvas.draw(armor);
        for (const auto& lightbar : result.lightbars)
            canvas.draw(lightbar);
        for (const auto& area : result.areas)
            canvas.draw(area);
        if (result.green_light) {
            canvas.draw(*result.green_light);
            canvas.draw(Canvas::Text {
                .content  = "GREEN LIGHT",
                .top_left = result.green_light->tl(),
                .color    = kGreen,
            });
        }
        for (const auto& icon : result.icons) {
            const auto center = icon.center.make<cv::Point2i>();
            canvas.draw(Canvas::Point { .origin = center, .radius = 5, .color = color });
            canvas.draw(Canvas::Text {
                .content  = std::format("R {}: {:.3f}", rune_color, icon.score),
                .top_left = center,
                .color    = color,
            });
        }
        for (const auto& bullseye : result.bullseyes) {
            if (!bullseye.active) {
                for (const auto& corner : bullseye.corners) {
                    canvas.draw(Canvas::Point {
                        .origin = corner.make<cv::Point2i>(),
                        .radius = 3,
                        .color  = color,
                    });
                }
            }
            const auto center = bullseye.center.make<cv::Point2i>();
            const auto state  = bullseye.active ? "ACTIVE" : "INACTIVE";
            canvas.draw(Canvas::Point { .origin = center, .radius = 5, .color = color });
            canvas.draw(Canvas::Text {
                .content  = std::format("B {} {}: {:.3f}", rune_color, state, bullseye.score),
                .top_left = center,
                .color    = color,
            });
        }
    }

private: // constants
    static constexpr auto kCapturerUnavailable = "The tunable capturer is unavailable";

    const rclcpp::Logger logger_ = get_logger();
    const std::string capturer_component_ =
        get_parameter_or<std::string>("capturer_component", "auto_aim_capturer");

private: // control
    util::ControlChannel control_channel_ { "/tmp/autoaim/detection", capturer_component_ };
    std::chrono::steady_clock::time_point next_publish_time_ { };

private: // state, guarded by mutex_
    std::mutex mutex_;
    std::condition_variable work_changed_;
    State state_;
    bool model_ready_         = false;
    std::uint64_t generation_ = 0;
    std::shared_ptr<const rmcs_msgs::CameraFrame> latest_frame_;

private: // flags
    std::atomic<bool> available_ { false };
    std::atomic<bool> stop_requested_ { false };

private: // io
    rmcs_utility::PooledSharedFactory<rmcs_msgs::CameraFrame> frame_factory_ { 8 };
    InputInterface<bool> available_input_;
    EventOutputInterface<std::shared_ptr<const rmcs_msgs::CameraFrame>> frame_output_;
    EventInputInterface<std::shared_ptr<const rmcs_msgs::CameraFrame>> frame_input_ {
        [this](const std::shared_ptr<const rmcs_msgs::CameraFrame>& frame) { handle_frame(frame); },
    };

private: // worker
    std::thread inference_thread_;
    std::thread service_thread_;
};

} // namespace rmcs

PLUGINLIB_EXPORT_CLASS(rmcs::AutoAimDetectionPreviewComponent, rmcs_executor::Component)
