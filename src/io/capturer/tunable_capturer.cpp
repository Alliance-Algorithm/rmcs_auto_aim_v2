#include <algorithm>
#include <atomic>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <experimental/scope>
#include <format>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>

#include <MvCameraControl.h>
#include <opencv2/imgproc.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/utilities.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/camera_frame.hpp>
#include <rmcs_utility/pooled_shared_factory.hpp>

#include "imu_snapshot_buffer.hpp"
#include "utility/control_channel.hpp"

namespace rmcs {

// Optional free-running capture path; the original synchronized capturer stays intact.
class AutoAimTunableCapturerComponent final : public rmcs_executor::Component, public rclcpp::Node {
public:
    AutoAimTunableCapturerComponent()
        : Node(get_component_name(),
              rclcpp::NodeOptions { }.automatically_declare_parameters_from_overrides(true))
        , camera_config_ {
            .camera_name = get_parameter_or<std::string>("camera_name", ""),
            .exposure_us = parameter_float(
                get_parameter_or<double>("exposure_us", 2000.0), "exposure_us", true),
            .gain = parameter_float(get_parameter_or<double>("gain", 10.0), "gain"),
            .framerate =
                parameter_float(get_parameter_or<double>("framerate", 120.0), "framerate", true),
            .invert_image = get_parameter_or<bool>("invert_image", false),
        } {
        if (get_parameter_or<bool>("use_hardware_sync", false)) {
            throw std::runtime_error {
                "AutoAimTunableCapturerComponent supports use_hardware_sync: false only",
            };
        }
        const auto frame_topic =
            get_parameter_or<std::string>("frame_topic", "/gimbal/auto_aim/camera_frame");
        if (frame_topic.empty()) throw std::runtime_error { "frame_topic must not be empty" };

        using Milliseconds   = std::chrono::duration<double, std::milli>;
        const auto delay_ms  = get_parameter_or<double>("delay_ms", 0.0);
        const auto max_delay = Milliseconds { std::chrono::steady_clock::duration::max() }.count();
        if (!std::isfinite(delay_ms) || std::abs(delay_ms) >= max_delay) {
            throw std::runtime_error { "delay_ms must be finite and fit the host clock" };
        }
        imu_delay_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            Milliseconds { delay_ms });

        const auto imu_snapshot_topic = get_parameter_or<std::string>("imu_snapshot_topic", "");
        if (!imu_snapshot_topic.empty()) {
            imu_buffer_ = std::make_unique<ImuSnapshotBuffer>(*this, 1024, imu_snapshot_topic);
        }

        register_output(frame_topic, frame_output_);
        register_output(std::format("/{}/tunable_capture_available", get_component_name()),
            available_output_, false);
        register_input(std::format("/{}/enable", get_component_name()), enable_input_, false);

        publish_state(true);
    }

    ~AutoAimTunableCapturerComponent() override {
        stop_requested_.store(true, std::memory_order::relaxed);
        if (worker_thread_.joinable()) worker_thread_.join();
    }

    void before_updating() override {
        update();
        worker_thread_ = std::thread { [this] { worker_main(); } };
        RCLCPP_INFO(logger_, "Tunable camera ready: %s/", control_channel_.directory().c_str());
    }

    void update() override {
        // Read executor-owned inputs only on its thread; the camera worker consumes this snapshot.
        const auto enabled = !enable_input_.ready() || *enable_input_;
        capture_enabled_.store(enabled, std::memory_order::relaxed);
        *available_output_ = enabled && camera_connected_.load(std::memory_order::relaxed);
    }

private:
    struct CameraConfig {
        std::string camera_name;
        float exposure_us;
        float gain;
        float framerate;
        bool invert_image;
    };

    struct FloatRange {
        float value;
        float min;
        float max;
    };

    struct RequestResult {
        std::string id;
        std::string parameter;
        bool ok = false;
        std::string error;
    };

    void worker_main() {
        try {
            while (!stop_requested_.load(std::memory_order::relaxed)) {
                control_channel_.drain_commands(
                    [this](const std::string& line) { handle_command(line); });
                if (!camera_connected_.load(std::memory_order::relaxed)
                    && std::chrono::steady_clock::now() >= next_connect_time_) {
                    connect_camera();
                }
                if (camera_connected_.load(std::memory_order::relaxed)) {
                    capture_frame();
                } else {
                    control_channel_.wait_for_command(std::chrono::milliseconds { 50 });
                }
                publish_state();
            }
            close_camera();
            camera_connected_.store(false, std::memory_order::relaxed);
            camera_error_ = "Camera component stopped";
            publish_state(true);
        } catch (const std::exception& error) {
            close_camera();
            camera_connected_.store(false, std::memory_order::relaxed);
            camera_error_ = error.what();
            RCLCPP_ERROR(logger_, "Tunable camera worker stopped: %s", error.what());
            try {
                publish_state(true);
            } catch (...) { }
            rclcpp::shutdown(
                get_node_base_interface()->get_context(), "Tunable camera worker stopped");
        }
    }

    void connect_camera() {
        try {
            open_camera();
            refresh_camera_ranges();
            camera_config_.exposure_us = exposure_us_range_->value;
            camera_config_.gain        = gain_range_->value;
            camera_connected_.store(true, std::memory_order::relaxed);
            camera_error_.clear();
            last_frame_time_ = std::chrono::steady_clock::now();
            RCLCPP_INFO(logger_, "[CAMERA] Tunable camera connected");
            publish_state(true);
        } catch (const std::exception& error) {
            disconnect_camera(error.what());
        }
    }

    void capture_frame() {
        const auto frame = capture_enabled_.load(std::memory_order::relaxed)
            ? frame_factory_.try_make()
            : std::shared_ptr<rmcs_msgs::CameraFrame> { };
        try {
            if (read_camera_frame(frame.get())) {
                last_frame_time_ = std::chrono::steady_clock::now();
                if (frame && capture_enabled_.load(std::memory_order::relaxed)) {
                    fill_and_emit(frame);
                }
                return;
            }
            const auto watchdog_timeout = std::chrono::duration<double> { std::max(
                5.0, 2.0 * (1.0 / camera_config_.framerate + camera_config_.exposure_us * 1e-6)) };
            if (std::chrono::steady_clock::now() - last_frame_time_ > watchdog_timeout) {
                disconnect_camera("Camera frame watchdog expired; reconnecting");
            }
        } catch (const std::exception& error) {
            disconnect_camera(error.what());
        }
    }

    void fill_and_emit(const std::shared_ptr<rmcs_msgs::CameraFrame>& frame) {
        using Rep           = std::chrono::steady_clock::duration::rep;
        const auto received = frame->image_reception_timestamp.time_since_epoch().count();
        const auto delay    = imu_delay_.count();
        if ((delay > 0 && received < std::numeric_limits<Rep>::min() + delay)
            || (delay < 0 && received > std::numeric_limits<Rep>::max() + delay)) {
            throw std::runtime_error { "delay_ms would overflow the image timestamp" };
        }
        frame->exposure_timestamp = std::chrono::steady_clock::time_point {
            std::chrono::steady_clock::duration { received - delay }
        };

        if (imu_buffer_) {
            const auto snapshot = imu_buffer_->pop_host(frame->exposure_timestamp);
            if (!snapshot) return;
            frame->imu_snapshot = snapshot->orientation;
            frame->gyro_body    = snapshot->gyro_body;
        } else {
            frame->imu_snapshot = Eigen::Quaterniond::Identity();
            frame->gyro_body    = Eigen::Vector3d::Zero();
        }
        frame->sync_publish_timestamp = std::chrono::steady_clock::now();
        frame_output_.emit(frame);
    }

    void open_camera() {
        // Like the old capturer, never finalize SDK resources owned by other components.
        static std::mutex open_mutex;
        const auto guard = std::lock_guard { open_mutex };

        auto devices = MV_CC_DEVICE_INFO_LIST { };
        check_sdk("Enumerate cameras", MV_CC_EnumDevices(MV_USB_DEVICE | MV_GIGE_DEVICE, &devices));
        auto* device = select_camera(devices, camera_config_.camera_name);
        if (!device) throw std::runtime_error { "No camera matched camera_name" };

        try {
            check_sdk("Create camera", MV_CC_CreateHandleWithoutLog(&camera_handle_, device));
            check_sdk("Open camera", MV_CC_OpenDevice(camera_handle_));
            if (device->nTLayerType == MV_GIGE_DEVICE) {
                const auto packet_size = MV_CC_GetOptimalPacketSize(camera_handle_);
                if (packet_size <= 0) {
                    throw std::runtime_error { "Cannot determine GigE packet size" };
                }
                check_sdk("Set GigE packet size",
                    MV_CC_SetIntValueEx(camera_handle_, "GevSCPSPacketSize", packet_size));
            }
            check_sdk("Set continuous acquisition",
                MV_CC_SetEnumValueByString(camera_handle_, "AcquisitionMode", "Continuous"));
            check_sdk("Disable trigger",
                MV_CC_SetEnumValueByString(camera_handle_, "TriggerMode", "Off"));
            check_sdk("Disable auto exposure",
                MV_CC_SetEnumValueByString(camera_handle_, "ExposureAuto", "Off"));
            check_sdk(
                "Disable auto gain", MV_CC_SetEnumValueByString(camera_handle_, "GainAuto", "Off"));
            check_sdk("Set ADC bit depth", MV_CC_SetEnumValue(camera_handle_, "ADCBitDepth", 0));
            check_sdk("Set image offset X", MV_CC_SetIntValueEx(camera_handle_, "OffsetX", 0));
            check_sdk("Set image offset Y", MV_CC_SetIntValueEx(camera_handle_, "OffsetY", 0));
            check_sdk("Set image width",
                MV_CC_SetIntValueEx(camera_handle_, "Width", rmcs_msgs::CameraFrame::kWidth));
            check_sdk("Set image height",
                MV_CC_SetIntValueEx(camera_handle_, "Height", rmcs_msgs::CameraFrame::kHeight));
            check_sdk("Set image reverse X",
                MV_CC_SetBoolValue(camera_handle_, "ReverseX", camera_config_.invert_image));
            check_sdk("Set image reverse Y",
                MV_CC_SetBoolValue(camera_handle_, "ReverseY", camera_config_.invert_image));
            check_sdk("Enable frame rate",
                MV_CC_SetBoolValue(camera_handle_, "AcquisitionFrameRateEnable", true));
            check_sdk("Set frame rate",
                MV_CC_SetFloatValue(
                    camera_handle_, "AcquisitionFrameRate", camera_config_.framerate));
            set_camera_value("ExposureTime", camera_config_.exposure_us);
            set_camera_value("Gain", camera_config_.gain);
            check_sdk("Start acquisition", MV_CC_StartGrabbing(camera_handle_));
        } catch (...) {
            close_camera();
            throw;
        }
    }

    void close_camera() noexcept {
        if (!camera_handle_) return;
        (void)MV_CC_StopGrabbing(camera_handle_);
        (void)MV_CC_CloseDevice(camera_handle_);
        (void)MV_CC_DestroyHandle(camera_handle_);
        camera_handle_ = nullptr;
    }

    [[nodiscard]] auto read_camera_frame(rmcs_msgs::CameraFrame* output) -> bool {
        auto frame             = MV_FRAME_OUT { };
        const auto code        = MV_CC_GetImageBuffer(camera_handle_, &frame, 50);
        const auto received_at = std::chrono::steady_clock::now();
        if (static_cast<std::uint32_t>(code) == MV_E_NODATA) {
            if (!MV_CC_IsDeviceConnected(camera_handle_)) {
                throw std::runtime_error { "Camera disconnected" };
            }
            return false;
        }
        check_sdk("Read image", code);
        auto release = std::experimental::scope_exit { [&] {
            (void)MV_CC_FreeImageBuffer(camera_handle_, &frame);
        } };

        // Always drain SDK frames, even when publishing is disabled or the pool is exhausted.
        if (!output) return true;
        const auto& info  = frame.stFrameInfo;
        const auto width  = info.nExtendWidth ? info.nExtendWidth : info.nWidth;
        const auto height = info.nExtendHeight ? info.nExtendHeight : info.nHeight;
        if (!frame.pBufAddr || width != output->kWidth || height != output->kHeight
            || info.nFrameLen != output->data_raw.size()) {
            throw std::runtime_error { "Expected a 1440x1080 Bayer8 camera frame" };
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
            throw std::runtime_error { "Camera pixel format must be Bayer8" };
        }
        std::memcpy(output->data_raw.data(), frame.pBufAddr, output->data_raw.size());
        output->image_reception_timestamp = received_at;

        const auto raw = cv::Mat { static_cast<int>(height), static_cast<int>(width), CV_8UC1,
            reinterpret_cast<unsigned char*>(output->data_raw.data()) };
        auto bgr       = cv::Mat { static_cast<int>(height), static_cast<int>(width), CV_8UC3,
            reinterpret_cast<unsigned char*>(output->data.data()) };
        cv::demosaicing(raw, bgr, output->opencv_cvt_color_code);
        return true;
    }

    [[nodiscard]] auto read_camera_range(const char* name) const -> FloatRange {
        auto value = MVCC_FLOATVALUE { };
        check_sdk(std::format("Read {}", name), MV_CC_GetFloatValue(camera_handle_, name, &value));
        if (!std::isfinite(value.fCurValue) || !std::isfinite(value.fMin)
            || !std::isfinite(value.fMax) || value.fMin > value.fMax || value.fCurValue < value.fMin
            || value.fCurValue > value.fMax) {
            throw std::runtime_error { std::format("Invalid camera range for {}", name) };
        }
        return { value.fCurValue, value.fMin, value.fMax };
    }

    void set_camera_value(const char* name, float value) {
        const auto limits = read_camera_range(name);
        if (!std::isfinite(value) || value < limits.min || value > limits.max) {
            throw std::runtime_error { std::format(
                "{} must be in [{}, {}]", name, limits.min, limits.max) };
        }
        check_sdk(std::format("Set {}", name), MV_CC_SetFloatValue(camera_handle_, name, value));
    }

    void refresh_camera_ranges() {
        const auto exposure_us = read_camera_range("ExposureTime");
        const auto gain        = read_camera_range("Gain");
        exposure_us_range_     = exposure_us;
        gain_range_            = gain;
    }

    void disconnect_camera(std::string message) {
        close_camera();
        camera_connected_.store(false, std::memory_order::relaxed);
        camera_error_      = std::move(message);
        next_connect_time_ = std::chrono::steady_clock::now() + std::chrono::seconds { 1 };
        RCLCPP_WARN(logger_, "[CAMERA] %s", camera_error_.c_str());
    }

    void publish_state(bool force = false) {
        const auto now = std::chrono::steady_clock::now();
        if (!force && now < next_publish_time_) return;
        next_publish_time_ = now + std::chrono::milliseconds { 100 };

        control_channel_.write_state(std::format(
            R"("component_type":{},"component_name":{},"connected":{},"error":{},)"
            R"("exposure_us":{},"gain":{},"last_request":{{"id":{},"parameter":{},"ok":{},)"
            R"("error":{}}})",
            util::json_string("rmcs::AutoAimTunableCapturerComponent"),
            util::json_string(get_component_name()),
            camera_connected_.load(std::memory_order::relaxed), util::json_string(camera_error_),
            range_json(exposure_us_range_), range_json(gain_range_),
            util::json_string(last_request_.id), util::json_string(last_request_.parameter),
            last_request_.ok, util::json_string(last_request_.error)));
    }

    void handle_command(const std::string& line) {
        auto input = std::istringstream { line };
        std::string id, parameter, number, extra;
        auto expires = std::int64_t { 0 };
        if (!(input >> id >> parameter >> number >> expires) || (input >> extra)) return;
        if (!util::ControlChannel::valid_request_id(id)) return;

        last_request_ =
            RequestResult { .id = id, .parameter = parameter, .ok = false, .error = { } };
        try {
            if (expires < util::unix_ms()) throw std::runtime_error { "Camera request expired" };
            if (parameter != "exposure_us" && parameter != "gain") {
                throw std::runtime_error { "Unknown camera parameter" };
            }
            const auto value = parameter_float(parse_number(number), parameter);
            if (!camera_connected_.load(std::memory_order::relaxed)) {
                throw std::runtime_error { "Camera is unavailable" };
            }
            const auto limits = parameter == "exposure_us" ? *exposure_us_range_ : *gain_range_;
            if (value < limits.min || value > limits.max) {
                throw std::runtime_error { std::format(
                    "Value must be in [{}, {}]", limits.min, limits.max) };
            }
            try {
                set_camera_value(parameter == "exposure_us" ? "ExposureTime" : "Gain", value);
                refresh_camera_ranges();
            } catch (const std::exception& error) {
                disconnect_camera(error.what());
                throw;
            }
            // Only confirmed values are reused on reconnect; never write the startup YAML.
            camera_config_.exposure_us = exposure_us_range_->value;
            camera_config_.gain        = gain_range_->value;
            last_request_.ok           = true;
            camera_error_.clear();
        } catch (const std::exception& error) {
            last_request_.error = error.what();
        }
        publish_state(true);
    }

    [[nodiscard]] static auto select_camera(const MV_CC_DEVICE_INFO_LIST& devices,
        const std::string& camera_name) -> MV_CC_DEVICE_INFO* {
        if (camera_name.empty()) {
            if (devices.nDeviceNum != 1) {
                throw std::runtime_error { std::format("Expected one camera, found {}; set "
                                                       "camera_name when multiple cameras exist",
                    devices.nDeviceNum) };
            }
            return devices.pDeviceInfo[0];
        }
        for (unsigned int i = 0; i < devices.nDeviceNum; ++i) {
            auto* device = devices.pDeviceInfo[i];
            if (!device) continue;
            const unsigned char* name = nullptr;
            if (device->nTLayerType == MV_USB_DEVICE) {
                name = device->SpecialInfo.stUsb3VInfo.chUserDefinedName;
            } else if (device->nTLayerType == MV_GIGE_DEVICE) {
                name = device->SpecialInfo.stGigEInfo.chUserDefinedName;
            }
            if (name && camera_name == reinterpret_cast<const char*>(name)) return device;
        }
        return nullptr;
    }

    [[nodiscard]] static auto parameter_float(
        double value, std::string_view name, bool positive = false) -> float {
        if (!std::isfinite(value) || std::abs(value) > std::numeric_limits<float>::max()) {
            throw std::runtime_error { std::format("{} must be a finite camera value", name) };
        }
        const auto result = static_cast<float>(value);
        if (positive && result <= 0) {
            throw std::runtime_error { std::format("{} must be positive", name) };
        }
        return result;
    }

    [[nodiscard]] static auto parse_number(std::string_view text) -> double {
        auto value        = 0.0;
        const auto result = std::from_chars(text.data(), text.data() + text.size(), value);
        if (result.ec != std::errc { } || result.ptr != text.data() + text.size()
            || !std::isfinite(value)) {
            throw std::runtime_error { "Expected a finite number" };
        }
        return value;
    }

    static void check_sdk(std::string_view action, int code) {
        if (code != MV_OK) {
            throw std::runtime_error { std::format(
                "{}: MVS error 0x{:08x}", action, static_cast<std::uint32_t>(code)) };
        }
    }

    [[nodiscard]] static auto range_json(const std::optional<FloatRange>& range) -> std::string {
        if (!range) return "null";
        return std::format(
            R"({{"value":{},"min":{},"max":{}}})", range->value, range->min, range->max);
    }

private: // constants
    const rclcpp::Logger logger_ = get_logger();

private: // camera
    CameraConfig camera_config_;
    void* camera_handle_ = nullptr;
    std::chrono::steady_clock::time_point next_connect_time_ { };
    std::chrono::steady_clock::time_point last_frame_time_ { };

private: // state
    std::atomic<bool> camera_connected_ { false };
    std::string camera_error_ = "Waiting for camera";
    std::optional<FloatRange> exposure_us_range_;
    std::optional<FloatRange> gain_range_;
    RequestResult last_request_;

private: // control
    util::ControlChannel control_channel_ { "/tmp/autoaim/camera", get_component_name() };
    std::chrono::steady_clock::time_point next_publish_time_ { };

private: // io
    std::chrono::steady_clock::duration imu_delay_ { };
    std::unique_ptr<ImuSnapshotBuffer> imu_buffer_;
    InputInterface<bool> enable_input_;
    OutputInterface<bool> available_output_;
    EventOutputInterface<std::shared_ptr<const rmcs_msgs::CameraFrame>> frame_output_;
    rmcs_utility::PooledSharedFactory<rmcs_msgs::CameraFrame> frame_factory_ { 128 };

private: // worker
    std::atomic<bool> capture_enabled_ { true };
    std::atomic<bool> stop_requested_ { false };
    std::thread worker_thread_;
};

} // namespace rmcs

PLUGINLIB_EXPORT_CLASS(rmcs::AutoAimTunableCapturerComponent, rmcs_executor::Component)
