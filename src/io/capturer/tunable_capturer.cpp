#include <algorithm>
#include <atomic>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <format>
#include <fstream>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
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

#include <MvCameraControl.h>
#include <opencv2/imgproc.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/utilities.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/camera_frame.hpp>
#include <rmcs_utility/pooled_shared_factory.hpp>

#include "imu_snapshot_buffer.hpp"

namespace rmcs {
namespace {

using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;
namespace fs = std::filesystem;

struct Config {
    std::string camera_name;
    float exposure_us = 2000.F;
    float gain = 10.F;
    float framerate = 120.F;
    bool invert_image = false;
};

auto unix_ms() -> std::int64_t {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

auto check_sdk(std::string_view action, int code) -> void {
    if (code != MV_OK) {
        throw std::runtime_error(std::format("{}: MVS error 0x{:08x}", action,
            static_cast<std::uint32_t>(code)));
    }
}

auto parse_number(std::string_view text) -> double {
    double value = 0;
    const auto result = std::from_chars(text.data(), text.data() + text.size(), value);
    if (result.ec != std::errc {} || result.ptr != text.data() + text.size()
        || !std::isfinite(value)) {
        throw std::runtime_error("Expected a finite number");
    }
    return value;
}

auto json_string(std::string_view text) -> std::string {
    std::string result = "\"";
    for (const unsigned char ch : text) {
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

struct FloatRange {
    float value;
    float min;
    float max;
};

struct Result {
    std::string id;
    std::string parameter;
    bool ok = false;
    std::string error;
};

struct State {
    bool connected = false;
    std::string error = "Waiting for camera";
    std::optional<FloatRange> exposure_us;
    std::optional<FloatRange> gain;
    Result last_request;
};

auto range_json(const std::optional<FloatRange>& range) -> std::string {
    if (!range) return "null";
    return std::format("{{\"value\":{},\"min\":{},\"max\":{}}}",
        range->value, range->min, range->max);
}

// Each component owns its own control directory; the legacy capturer is unchanged.
class Control {
public:
    explicit Control(std::string_view component_name)
        : directory_(fs::path("/tmp/autoaim/camera") / std::string(component_name)) {
        const auto valid_start = [](unsigned char ch) {
            return (ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') || ch == '_';
        };
        if (component_name.empty() || !valid_start(component_name.front())
            || !std::ranges::all_of(component_name, [&](unsigned char ch) {
                   return valid_start(ch) || (ch >= '0' && ch <= '9');
               })) {
            throw std::runtime_error("Invalid camera control component name");
        }
        try {
            fs::create_directories(directory_);
            lock_fd_ = ::open((directory_ / ".lock").c_str(), O_CREAT | O_RDWR | O_CLOEXEC, 0644);
            if (lock_fd_ < 0 || ::flock(lock_fd_, LOCK_EX | LOCK_NB) != 0) {
                throw std::runtime_error("Cannot lock camera control directory; already running?");
            }
            owns_lock_ = true;
            const auto path = directory_ / "commands";
            struct stat info {};
            if (::lstat(path.c_str(), &info) == 0) {
                if (!S_ISFIFO(info.st_mode)) {
                    throw std::runtime_error("camera control commands path is not a FIFO");
                }
                fs::remove(path);
            }
            if (::mkfifo(path.c_str(), 0660) != 0) {
                throw std::runtime_error("Cannot create camera control commands FIFO");
            }
            owns_fifo_ = true;
            fifo_fd_ = ::open(path.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
            if (fifo_fd_ < 0) throw std::runtime_error("Cannot open camera control commands FIFO");
        } catch (...) {
            cleanup();
            throw;
        }
    }

    ~Control() { cleanup(); }
    Control(const Control&) = delete;
    auto operator=(const Control&) -> Control& = delete;

    auto publish(const State& state, bool force = false) -> void {
        const auto now = Clock::now();
        if (!force && now < next_publish_) return;
        next_publish_ = now + 100ms;
        const auto& result = state.last_request;
        const auto json = std::format(
            "{{\"pid\":{},\"updated_at\":{},\"connected\":{},\"error\":{},"
            "\"exposure_us\":{},\"gain\":{},\"last_request\":{{\"id\":{},"
            "\"parameter\":{},\"ok\":{},\"error\":{}}}}}\n",
            ::getpid(), static_cast<double>(unix_ms()) / 1000.0, state.connected,
            json_string(state.error), range_json(state.exposure_us), range_json(state.gain),
            json_string(result.id), json_string(result.parameter), result.ok,
            json_string(result.error));
        const auto temporary = directory_ / "state.json.tmp";
        {
            auto output = std::ofstream(temporary, std::ios::trunc);
            output << json;
            output.close();
            if (!output) throw std::runtime_error("Cannot write camera control state");
        }
        fs::rename(temporary, directory_ / "state.json");
    }

    template <typename Callback>
    auto drain(Callback&& callback) -> void {
        char buffer[512];
        // Bound work per frame even if a writer keeps the pipe continuously busy.
        for (int reads = 0; reads < 8; ++reads) {
            const auto count = ::read(fifo_fd_, buffer, sizeof(buffer));
            if (count <= 0) break;
            for (std::ptrdiff_t i = 0; i < count; ++i) {
                const auto ch = buffer[i];
                if (ch == '\n') {
                    if (!discard_line_ && !pending_.empty()) callback(pending_);
                    pending_.clear();
                    discard_line_ = false;
                } else if (!discard_line_) {
                    if (pending_.size() >= 256) {
                        pending_.clear();
                        discard_line_ = true;
                    } else {
                        pending_ += ch;
                    }
                }
            }
        }
    }

    auto wait(int milliseconds) const -> void {
        auto descriptor = pollfd { fifo_fd_, POLLIN, 0 };
        (void)::poll(&descriptor, 1, milliseconds);
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
        // Keep the lock inode so a new owner cannot race an unlinked lock file.
        if (lock_fd_ >= 0) ::close(lock_fd_);
        fifo_fd_ = lock_fd_ = -1;
        owns_lock_ = false;
    }

    const fs::path directory_;
    int lock_fd_ = -1;
    int fifo_fd_ = -1;
    bool owns_lock_ = false;
    bool owns_fifo_ = false;
    bool discard_line_ = false;
    std::string pending_;
    Clock::time_point next_publish_ {};
};

class Camera {
public:
    ~Camera() { close(); }
    Camera() = default;
    Camera(const Camera&) = delete;
    auto operator=(const Camera&) -> Camera& = delete;

    auto close() noexcept -> void {
        if (!handle_) return;
        (void)MV_CC_StopGrabbing(handle_);
        (void)MV_CC_CloseDevice(handle_);
        (void)MV_CC_DestroyHandle(handle_);
        handle_ = nullptr;
    }

    auto open(const Config& config) -> void {
        // Match the old capturer's SDK lifetime: never finalize SDK resources owned by other components.
        static std::mutex open_mutex;
        const auto guard = std::lock_guard(open_mutex);
        auto devices = MV_CC_DEVICE_INFO_LIST {};
        check_sdk("Enumerate cameras", MV_CC_EnumDevices(MV_USB_DEVICE | MV_GIGE_DEVICE, &devices));
        MV_CC_DEVICE_INFO* selected = nullptr;
        if (config.camera_name.empty()) {
            if (devices.nDeviceNum != 1) {
                throw std::runtime_error(std::format(
                    "Expected one camera, found {}; set camera_name when multiple cameras exist",
                    devices.nDeviceNum));
            }
            selected = devices.pDeviceInfo[0];
        } else {
            for (unsigned int i = 0; i < devices.nDeviceNum; ++i) {
                auto* device = devices.pDeviceInfo[i];
                if (!device) continue;
                const unsigned char* name = nullptr;
                if (device->nTLayerType == MV_USB_DEVICE) {
                    name = device->SpecialInfo.stUsb3VInfo.chUserDefinedName;
                } else if (device->nTLayerType == MV_GIGE_DEVICE) {
                    name = device->SpecialInfo.stGigEInfo.chUserDefinedName;
                }
                if (name && config.camera_name == reinterpret_cast<const char*>(name)) {
                    selected = device;
                    break;
                }
            }
        }
        if (!selected) throw std::runtime_error("No camera matched camera_name");
        try {
            check_sdk("Create camera", MV_CC_CreateHandleWithoutLog(&handle_, selected));
            check_sdk("Open camera", MV_CC_OpenDevice(handle_));
            if (selected->nTLayerType == MV_GIGE_DEVICE) {
                const auto packet_size = MV_CC_GetOptimalPacketSize(handle_);
                if (packet_size <= 0) throw std::runtime_error("Cannot determine GigE packet size");
                check_sdk("Set GigE packet size",
                    MV_CC_SetIntValueEx(handle_, "GevSCPSPacketSize", packet_size));
            }
            check_sdk("Set continuous acquisition",
                MV_CC_SetEnumValueByString(handle_, "AcquisitionMode", "Continuous"));
            check_sdk("Disable trigger", MV_CC_SetEnumValueByString(handle_, "TriggerMode", "Off"));
            check_sdk("Disable auto exposure",
                MV_CC_SetEnumValueByString(handle_, "ExposureAuto", "Off"));
            check_sdk("Disable auto gain", MV_CC_SetEnumValueByString(handle_, "GainAuto", "Off"));
            check_sdk("Set ADC bit depth", MV_CC_SetEnumValue(handle_, "ADCBitDepth", 0));
            check_sdk("Set image offset X", MV_CC_SetIntValueEx(handle_, "OffsetX", 0));
            check_sdk("Set image offset Y", MV_CC_SetIntValueEx(handle_, "OffsetY", 0));
            check_sdk("Set image width", MV_CC_SetIntValueEx(handle_, "Width", rmcs_msgs::CameraFrame::kWidth));
            check_sdk("Set image height", MV_CC_SetIntValueEx(handle_, "Height", rmcs_msgs::CameraFrame::kHeight));
            check_sdk("Set image reverse X", MV_CC_SetBoolValue(handle_, "ReverseX", config.invert_image));
            check_sdk("Set image reverse Y", MV_CC_SetBoolValue(handle_, "ReverseY", config.invert_image));
            check_sdk("Enable frame rate", MV_CC_SetBoolValue(handle_, "AcquisitionFrameRateEnable", true));
            check_sdk("Set frame rate", MV_CC_SetFloatValue(handle_, "AcquisitionFrameRate", config.framerate));
            set("ExposureTime", config.exposure_us);
            set("Gain", config.gain);
            check_sdk("Start acquisition", MV_CC_StartGrabbing(handle_));
        } catch (...) {
            close();
            throw;
        }
    }

    auto range(const char* name) const -> FloatRange {
        auto value = MVCC_FLOATVALUE {};
        check_sdk(std::format("Read {}", name), MV_CC_GetFloatValue(handle_, name, &value));
        if (!std::isfinite(value.fCurValue) || !std::isfinite(value.fMin)
            || !std::isfinite(value.fMax) || value.fMin > value.fMax
            || value.fCurValue < value.fMin || value.fCurValue > value.fMax) {
            throw std::runtime_error(std::format("Invalid camera range for {}", name));
        }
        return { value.fCurValue, value.fMin, value.fMax };
    }

    auto set(const char* name, float value) -> void {
        const auto limits = range(name);
        if (!std::isfinite(value) || value < limits.min || value > limits.max) {
            throw std::runtime_error(std::format("{} must be in [{}, {}]", name, limits.min, limits.max));
        }
        check_sdk(std::format("Set {}", name), MV_CC_SetFloatValue(handle_, name, value));
    }

    auto read(rmcs_msgs::CameraFrame* output) -> bool {
        auto frame = MV_FRAME_OUT {};
        const auto code = MV_CC_GetImageBuffer(handle_, &frame, 50);
        const auto received_at = Clock::now();
        if (static_cast<std::uint32_t>(code) == MV_E_NODATA) {
            if (!MV_CC_IsDeviceConnected(handle_)) throw std::runtime_error("Camera disconnected");
            return false;
        }
        check_sdk("Read image", code);
        struct Release {
            void* handle;
            MV_FRAME_OUT& frame;
            ~Release() { (void)MV_CC_FreeImageBuffer(handle, &frame); }
        } release { handle_, frame };

        // Always drain SDK frames, even when publishing is disabled or the pool is exhausted.
        if (!output) return true;
        const auto& info = frame.stFrameInfo;
        const auto width = info.nExtendWidth ? info.nExtendWidth : info.nWidth;
        const auto height = info.nExtendHeight ? info.nExtendHeight : info.nHeight;
        if (!frame.pBufAddr || width != output->kWidth || height != output->kHeight
            || info.nFrameLen != output->data_raw.size()) {
            throw std::runtime_error("Expected a 1440x1080 Bayer8 camera frame");
        }
        switch (info.enPixelType) {
        case PixelType_Gvsp_BayerRG8:
            output->opencv_cvt_color_code = cv::COLOR_BayerRGGB2BGR_EA;
            break;
        case PixelType_Gvsp_BayerBG8:
            output->opencv_cvt_color_code = cv::COLOR_BayerBGGR2BGR_EA;
            break;
        case PixelType_Gvsp_BayerGR8:
            output->opencv_cvt_color_code = cv::COLOR_BayerGRBG2BGR_EA;
            break;
        case PixelType_Gvsp_BayerGB8:
            output->opencv_cvt_color_code = cv::COLOR_BayerGBRG2BGR_EA;
            break;
        default:
            throw std::runtime_error("Camera pixel format must be Bayer8");
        }
        std::memcpy(output->data_raw.data(), frame.pBufAddr, output->data_raw.size());
        output->image_reception_timestamp = received_at;
        const auto raw = cv::Mat(static_cast<int>(height), static_cast<int>(width), CV_8UC1,
            reinterpret_cast<unsigned char*>(output->data_raw.data()));
        auto bgr = cv::Mat(static_cast<int>(height), static_cast<int>(width), CV_8UC3,
            reinterpret_cast<unsigned char*>(output->data.data()));
        cv::demosaicing(raw, bgr, output->opencv_cvt_color_code);
        return true;
    }

private:
    void* handle_ = nullptr;
};


auto parameter_float(double value, std::string_view name, bool positive = false) -> float {
    if (!std::isfinite(value) || std::abs(value) > std::numeric_limits<float>::max()) {
        throw std::runtime_error(std::format("{} must be a finite camera value", name));
    }
    const auto result = static_cast<float>(value);
    if (positive && result <= 0) throw std::runtime_error(std::format("{} must be positive", name));
    return result;
}

} // namespace

// Optional free-running capture path; the original synchronized capturer stays intact.
class AutoAimTunableCapturerComponent final : public rmcs_executor::Component, public rclcpp::Node {
public:
    AutoAimTunableCapturerComponent()
        : Node(get_component_name(),
              rclcpp::NodeOptions {}.automatically_declare_parameters_from_overrides(true))
        , config_ {
            .camera_name = get_parameter_or<std::string>("camera_name", ""),
            .exposure_us = parameter_float(get_parameter_or<double>("exposure_us", 2000.0), "exposure_us", true),
            .gain = parameter_float(get_parameter_or<double>("gain", 10.0), "gain"),
            .framerate = parameter_float(get_parameter_or<double>("framerate", 120.0), "framerate", true),
            .invert_image = get_parameter_or<bool>("invert_image", false),
        } {
        if (get_parameter_or<bool>("use_hardware_sync", false)) {
            throw std::runtime_error("AutoAimTunableCapturerComponent supports use_hardware_sync: false only");
        }
        const auto topic = get_parameter_or<std::string>("frame_topic", "/gimbal/auto_aim/camera_frame");
        if (topic.empty()) throw std::runtime_error("frame_topic must not be empty");
        const auto delay_ms = get_parameter_or<double>("delay_ms", 0.0);
        const auto max_delay = std::chrono::duration<double, std::milli>(Clock::duration::max()).count();
        if (!std::isfinite(delay_ms) || std::abs(delay_ms) >= max_delay) {
            throw std::runtime_error("delay_ms must be finite and fit the host clock");
        }
        imu_delay_ = std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double, std::milli>(delay_ms));
        const auto imu_topic = get_parameter_or<std::string>("imu_snapshot_topic", "");
        if (!imu_topic.empty()) imu_buffer_ = std::make_unique<ImuSnapshotBuffer>(*this, 1024, imu_topic);
        register_output(topic, frame_output_);
        register_input(std::format("/{}/enable", get_component_name()), enable_input_, false);
        control_ = std::make_unique<Control>(get_component_name());
        control_->publish(state_, true);
    }

    ~AutoAimTunableCapturerComponent() override {
        stop_requested_.store(true, std::memory_order_relaxed);
        if (worker_.joinable()) worker_.join();
    }

    auto before_updating() -> void override {
        update();
        worker_ = std::thread([this] { worker_main(); });
        RCLCPP_INFO(get_logger(), "Tunable camera ready: /tmp/autoaim/camera/%s/",
            get_component_name().c_str());
    }

    auto update() -> void override {
        // Read executor-owned inputs only on its thread; the camera worker consumes this snapshot.
        capture_enabled_.store(!enable_input_.ready() || *enable_input_, std::memory_order_relaxed);
    }

private:
    auto refresh(Camera& camera) -> void {
        const auto exposure = camera.range("ExposureTime");
        const auto gain = camera.range("Gain");
        state_.exposure_us = exposure;
        state_.gain = gain;
    }

    auto disconnect(Camera& camera, std::string message) -> void {
        camera.close();
        state_.connected = false;
        state_.error = std::move(message);
        next_connect_ = Clock::now() + 1s;
        RCLCPP_WARN(get_logger(), "[CAMERA] %s", state_.error.c_str());
    }

    auto handle_command(Camera& camera, const std::string& line) -> void {
        std::istringstream input(line);
        std::string id, parameter, number, extra;
        std::int64_t expires = 0;
        if (!(input >> id >> parameter >> number >> expires) || (input >> extra)
            || id.size() > 64 || !std::ranges::all_of(id, [](unsigned char ch) {
                   return (ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z')
                       || (ch >= '0' && ch <= '9') || ch == '-';
               })) return;
        state_.last_request = { .id = id, .parameter = parameter, .ok = false, .error = {} };
        try {
            if (expires < unix_ms()) throw std::runtime_error("Camera request expired");
            if (parameter != "exposure_us" && parameter != "gain") {
                throw std::runtime_error("Unknown camera parameter");
            }
            const auto value = parameter_float(parse_number(number), parameter);
            if (!state_.connected) throw std::runtime_error("Camera is unavailable");
            const auto limits = parameter == "exposure_us" ? *state_.exposure_us : *state_.gain;
            if (value < limits.min || value > limits.max) {
                throw std::runtime_error(std::format("Value must be in [{}, {}]", limits.min, limits.max));
            }
            try {
                camera.set(parameter == "exposure_us" ? "ExposureTime" : "Gain", value);
                refresh(camera);
            } catch (const std::exception& error) {
                disconnect(camera, error.what());
                throw;
            }
            // Only confirmed values are reused on reconnect; never write the startup YAML.
            config_.exposure_us = state_.exposure_us->value;
            config_.gain = state_.gain->value;
            state_.last_request.ok = true;
            state_.error.clear();
        } catch (const std::exception& error) {
            state_.last_request.error = error.what();
        }
        control_->publish(state_, true);
    }

    auto fill_and_emit(const std::shared_ptr<rmcs_msgs::CameraFrame>& frame) -> void {
        const auto received = frame->image_reception_timestamp.time_since_epoch().count();
        const auto delay = imu_delay_.count();
        using Rep = Clock::duration::rep;
        if ((delay > 0 && received < std::numeric_limits<Rep>::min() + delay)
            || (delay < 0 && received > std::numeric_limits<Rep>::max() + delay)) {
            throw std::runtime_error("delay_ms would overflow the image timestamp");
        }
        frame->exposure_timestamp = Clock::time_point(Clock::duration(received - delay));
        if (imu_buffer_) {
            const auto snapshot = imu_buffer_->pop_host(frame->exposure_timestamp);
            if (!snapshot) return;
            frame->imu_snapshot = snapshot->orientation;
            frame->gyro_body = snapshot->gyro_body;
        } else {
            frame->imu_snapshot = Eigen::Quaterniond::Identity();
            frame->gyro_body = Eigen::Vector3d::Zero();
        }
        frame->sync_publish_timestamp = Clock::now();
        frame_output_.emit(frame);
    }

    auto worker_main() -> void {
        Camera camera;
        try {
            auto last_frame = Clock::now();
            while (!stop_requested_.load(std::memory_order_relaxed)) {
                control_->drain([&](const std::string& line) { handle_command(camera, line); });
                if (!state_.connected && Clock::now() >= next_connect_) {
                    try {
                        camera.open(config_);
                        refresh(camera);
                        config_.exposure_us = state_.exposure_us->value;
                        config_.gain = state_.gain->value;
                        state_.connected = true;
                        state_.error.clear();
                        last_frame = Clock::now();
                        RCLCPP_INFO(get_logger(), "[CAMERA] Tunable camera connected");
                        control_->publish(state_, true);
                    } catch (const std::exception& error) {
                        disconnect(camera, error.what());
                    }
                }
                if (state_.connected) {
                    const auto frame = capture_enabled_.load(std::memory_order_relaxed)
                        ? frame_factory_.try_make() : std::shared_ptr<rmcs_msgs::CameraFrame> {};
                    try {
                        if (camera.read(frame.get())) {
                            last_frame = Clock::now();
                            if (frame && capture_enabled_.load(std::memory_order_relaxed)) fill_and_emit(frame);
                        } else if (const auto timeout = std::chrono::duration<double>(std::max(5.0,
                                       2.0 * (1.0 / config_.framerate + config_.exposure_us * 1e-6)));
                            Clock::now() - last_frame > timeout) {
                            disconnect(camera, "Camera frame watchdog expired; reconnecting");
                        }
                    } catch (const std::exception& error) {
                        disconnect(camera, error.what());
                    }
                } else {
                    control_->wait(50);
                }
                control_->publish(state_);
            }
            camera.close();
            state_.connected = false;
            state_.error = "Camera component stopped";
            control_->publish(state_, true);
        } catch (const std::exception& error) {
            camera.close();
            state_.connected = false;
            state_.error = error.what();
            RCLCPP_ERROR(get_logger(), "Tunable camera worker stopped: %s", error.what());
            try { control_->publish(state_, true); } catch (...) {}
            rclcpp::shutdown(get_node_base_interface()->get_context(), "Tunable camera worker stopped");
        }
    }

    Config config_;
    State state_;
    std::unique_ptr<Control> control_;
    std::unique_ptr<ImuSnapshotBuffer> imu_buffer_;
    Clock::duration imu_delay_ {};
    Clock::time_point next_connect_ {};
    InputInterface<bool> enable_input_;
    EventOutputInterface<std::shared_ptr<const rmcs_msgs::CameraFrame>> frame_output_;
    rmcs_utility::PooledSharedFactory<rmcs_msgs::CameraFrame> frame_factory_ { 128 };
    std::atomic<bool> capture_enabled_ { true };
    std::atomic<bool> stop_requested_ { false };
    std::thread worker_;
};

} // namespace rmcs

PLUGINLIB_EXPORT_CLASS(rmcs::AutoAimTunableCapturerComponent, rmcs_executor::Component)
