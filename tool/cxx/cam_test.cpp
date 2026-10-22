#include <algorithm>
#include <charconv>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <cstdint>
#include <filesystem>
#include <format>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <syncstream>
#include <thread>

#include <fcntl.h>
#include <poll.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>

#include <MvCameraControl.h>

#include "module/debug/visualization/stream_context.hpp"

namespace {

using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;
namespace fs = std::filesystem;

volatile std::sig_atomic_t stop_requested = 0;

struct Config {
    std::string camera_name;
    float exposure_us = 2000.F;
    float gain = 10.F;
    float framerate = 120.F;
    int stream_framerate = 60;
};

auto unix_ms() -> std::int64_t {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

auto log(std::string_view message) -> void {
    std::osyncstream(std::cerr) << "[cam_test] " << message << '\n';
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

auto parse_config(int argc, char** argv) -> std::optional<Config> {
    Config config;
    for (int i = 1; i < argc; ++i) {
        const auto option = std::string_view(argv[i]);
        if (option == "--help" || option == "-h") {
            std::cout << "Usage: cam_test [--camera-name NAME] [--exposure-us 2000] [--gain 10]\n"
                         "                [--framerate 120] [--stream-framerate 60]\n"
                         "RTP: 127.0.0.1:5001; control: /tmp/autoaim/cam_test/\n"
                         "Start tool/res/start-cam-test separately; open http://<ip>:18081/\n";
            return std::nullopt;
        }
        if (++i >= argc) throw std::runtime_error("Missing value for " + std::string(option));
        if (option == "--camera-name") {
            config.camera_name = argv[i];
            continue;
        }
        const auto value = parse_number(argv[i]);
        if (std::abs(value) > std::numeric_limits<float>::max()) {
            throw std::runtime_error("Number exceeds the camera floating-point range");
        }
        if (option == "--exposure-us" && value > 0) {
            config.exposure_us = static_cast<float>(value);
        } else if (option == "--gain") {
            config.gain = static_cast<float>(value);
        } else if (option == "--framerate" && value > 0) {
            config.framerate = static_cast<float>(value);
        } else if (option == "--stream-framerate" && value >= 1 && value <= 1000
            && value == std::floor(value)) {
            config.stream_framerate = static_cast<int>(value);
        } else {
            throw std::runtime_error("Unknown option or invalid value: " + std::string(option));
        }
    }
    return config;
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

// This IPC belongs exclusively to the tool. No shared library protocol is changed.
class Control {
public:
    Control() {
        try {
            fs::create_directories(directory_);
            lock_fd_ = ::open((directory_ / ".lock").c_str(), O_CREAT | O_RDWR | O_CLOEXEC, 0644);
            if (lock_fd_ < 0 || ::flock(lock_fd_, LOCK_EX | LOCK_NB) != 0) {
                throw std::runtime_error("Cannot lock cam_test control directory; already running?");
            }
            owns_lock_ = true;
            const auto path = directory_ / "commands";
            struct stat info {};
            if (::lstat(path.c_str(), &info) == 0) {
                if (!S_ISFIFO(info.st_mode)) {
                    throw std::runtime_error("cam_test commands path is not a FIFO");
                }
                fs::remove(path);
            }
            if (::mkfifo(path.c_str(), 0660) != 0) {
                throw std::runtime_error("Cannot create cam_test commands FIFO");
            }
            owns_fifo_ = true;
            fifo_fd_ = ::open(path.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
            if (fifo_fd_ < 0) throw std::runtime_error("Cannot open cam_test commands FIFO");
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
            if (!output) throw std::runtime_error("Cannot write cam_test state");
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

    const fs::path directory_ = "/tmp/autoaim/cam_test";
    int lock_fd_ = -1;
    int fifo_fd_ = -1;
    bool owns_lock_ = false;
    bool owns_fifo_ = false;
    bool discard_line_ = false;
    std::string pending_;
    Clock::time_point next_publish_ {};
};

class SdkRuntime {
public:
    SdkRuntime() { check_sdk("Initialize SDK", MV_CC_Initialize()); }
    ~SdkRuntime() { (void)MV_CC_Finalize(); }
    SdkRuntime(const SdkRuntime&) = delete;
    auto operator=(const SdkRuntime&) -> SdkRuntime& = delete;
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
        auto devices = MV_CC_DEVICE_INFO_LIST {};
        check_sdk("Enumerate cameras", MV_CC_EnumDevices(MV_USB_DEVICE | MV_GIGE_DEVICE, &devices));
        MV_CC_DEVICE_INFO* selected = nullptr;
        if (config.camera_name.empty()) {
            if (devices.nDeviceNum != 1) {
                throw std::runtime_error(std::format(
                    "Expected one camera, found {}; use --camera-name when multiple cameras exist",
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
        if (!selected) throw std::runtime_error("No camera matched --camera-name");
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

    auto read() -> cv::Mat {
        auto frame = MV_FRAME_OUT {};
        const auto code = MV_CC_GetImageBuffer(handle_, &frame, 50);
        if (static_cast<std::uint32_t>(code) == MV_E_NODATA) {
            if (!MV_CC_IsDeviceConnected(handle_)) throw std::runtime_error("Camera disconnected");
            return {};
        }
        check_sdk("Read image", code);
        struct Release {
            void* handle;
            MV_FRAME_OUT& frame;
            ~Release() { (void)MV_CC_FreeImageBuffer(handle, &frame); }
        } release { handle_, frame };

        const auto& info = frame.stFrameInfo;
        const auto width = info.nExtendWidth ? info.nExtendWidth : info.nWidth;
        const auto height = info.nExtendHeight ? info.nExtendHeight : info.nHeight;
        const auto bytes = static_cast<std::uint64_t>(width) * height * 3;
        if (!frame.pBufAddr || width == 0 || height == 0
            || width > static_cast<unsigned int>(std::numeric_limits<int>::max())
            || height > static_cast<unsigned int>(std::numeric_limits<int>::max())
            || bytes > std::numeric_limits<unsigned int>::max()) {
            throw std::runtime_error("Invalid image dimensions");
        }
        auto image = cv::Mat(static_cast<int>(height), static_cast<int>(width), CV_8UC3);
        auto conversion = MV_CC_PIXEL_CONVERT_PARAM_EX {};
        conversion.nWidth = width;
        conversion.nHeight = height;
        conversion.enSrcPixelType = info.enPixelType;
        conversion.pSrcData = frame.pBufAddr;
        conversion.nSrcDataLen = info.nFrameLen;
        conversion.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
        conversion.pDstBuffer = image.data;
        conversion.nDstBufferSize = static_cast<unsigned int>(bytes);
        check_sdk("Convert image to BGR", MV_CC_ConvertPixelTypeEx(handle_, &conversion));
        if (conversion.nDstLen != bytes) throw std::runtime_error("Incomplete converted image");
        return image;
    }

private:
    void* handle_ = nullptr;
};

struct LatestFrame {
    std::mutex mutex;
    std::condition_variable_any changed;
    cv::Mat image;
    std::uint64_t generation = 0;

    auto update(cv::Mat next) -> void {
        {
            const auto lock = std::lock_guard(mutex);
            image = std::move(next);
            ++generation;
        }
        changed.notify_one();
    }
};

auto stream_frames(LatestFrame& frames, int hz, std::stop_token stop) -> void {
    try {
        using Stream = rmcs::debug::StreamContext;
        if (const auto support = Stream::check_support(); !support) {
            log(support.error());
            return;
        }
        std::unique_ptr<Stream> stream;
        std::uint64_t generation = 0;
        auto next_frame = Clock::now();
        auto next_open = Clock::now();
        const auto interval = std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(1.0 / hz));
        while (!stop.stop_requested()) {
            cv::Mat image;
            {
                auto lock = std::unique_lock(frames.mutex);
                if (!frames.changed.wait(lock, stop,
                        [&] { return frames.generation != generation; })) break;
                generation = frames.generation;
                image = frames.image;
            }
            if (image.empty() || Clock::now() < next_frame) continue;
            next_frame = Clock::now() + interval;
            if (stream && (stream->video_format().w != image.cols
                              || stream->video_format().h != image.rows)) {
                stream.reset();
            }
            if (!stream) {
                if (Clock::now() < next_open) continue;
                next_open = Clock::now() + 2s;
                auto candidate = std::make_unique<Stream>(Stream::StreamType::RTP_H264,
                    Stream::VideoFormat { image.cols, image.rows, hz },
                    Stream::StreamTarget { "127.0.0.1", "5001" });
                if (const auto result = candidate->open(); !result) {
                    log(result.error());
                    continue;
                }
                stream = std::move(candidate);
                log("Streaming H264 RTP to 127.0.0.1:5001");
            }
            stream->write(image);
        }
    } catch (const std::exception& error) {
        log(std::format("Streaming stopped: {}", error.what()));
    }
}

auto refresh(Camera& camera, State& state) -> void {
    const auto exposure = camera.range("ExposureTime");
    const auto gain = camera.range("Gain");
    state.exposure_us = exposure;
    state.gain = gain;
}

auto run(Config config) -> int {
    Control control;
    State state;
    control.publish(state, true);
    SdkRuntime sdk;
    Camera camera;
    LatestFrame frames;
    std::jthread streamer([&](std::stop_token stop) {
        stream_frames(frames, config.stream_framerate, stop);
    });
    auto next_connect = Clock::now();
    auto last_frame = Clock::now();

    const auto disconnect = [&](std::string message) {
        camera.close();
        state.connected = false;
        state.error = std::move(message);
        next_connect = Clock::now() + 1s;
        frames.update({});
        log(state.error);
    };

    const auto command = [&](const std::string& line) {
        std::istringstream input(line);
        std::string id, parameter, number, extra;
        std::int64_t expires = 0;
        if (!(input >> id >> parameter >> number >> expires) || (input >> extra)
            || id.size() > 64 || !std::ranges::all_of(id, [](unsigned char ch) {
                   return (ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z')
                       || (ch >= '0' && ch <= '9') || ch == '-';
               })) return;
        state.last_request = { .id = id, .parameter = parameter, .ok = false, .error = {} };
        try {
            if (expires < unix_ms()) throw std::runtime_error("Camera request expired");
            if (parameter != "exposure_us" && parameter != "gain") {
                throw std::runtime_error("Unknown camera parameter");
            }
            const auto value = parse_number(number);
            if (!state.connected) throw std::runtime_error("Camera is unavailable");
            const auto limits = parameter == "exposure_us" ? *state.exposure_us : *state.gain;
            if (std::abs(value) > std::numeric_limits<float>::max()) {
                throw std::runtime_error("Value exceeds the camera floating-point range");
            }
            // The SDK accepts floats. Round before checking its exact endpoints.
            const auto camera_value = static_cast<float>(value);
            if (camera_value < limits.min || camera_value > limits.max) {
                throw std::runtime_error(std::format("Value must be in [{}, {}]", limits.min, limits.max));
            }
            try {
                camera.set(parameter == "exposure_us" ? "ExposureTime" : "Gain", camera_value);
                refresh(camera, state);
            } catch (const std::exception& error) {
                // Restore the last confirmed configuration on reconnect if readback failed.
                disconnect(error.what());
                throw;
            }
            config.exposure_us = state.exposure_us->value;
            config.gain = state.gain->value;
            state.last_request.ok = true;
            state.error.clear();
        } catch (const std::exception& error) {
            state.last_request.error = error.what();
        }
        control.publish(state, true);
    };

    log("Camera control ready; start tool/res/start-cam-test and open http://<ip>:18081/");
    while (!stop_requested) {
        control.drain(command);
        if (!state.connected && Clock::now() >= next_connect) {
            try {
                camera.open(config);
                refresh(camera, state);
                config.exposure_us = state.exposure_us->value;
                config.gain = state.gain->value;
                state.connected = true;
                state.error.clear();
                last_frame = Clock::now();
                log("Camera connected");
                control.publish(state, true);
            } catch (const std::exception& error) {
                disconnect(error.what());
            }
        }
        if (state.connected) {
            try {
                auto image = camera.read();
                if (!image.empty()) {
                    last_frame = Clock::now();
                    frames.update(std::move(image));
                } else if (const auto timeout = std::chrono::duration<double>(std::max(5.0,
                               2.0 * (1.0 / config.framerate + config.exposure_us * 1e-6)));
                    Clock::now() - last_frame > timeout) {
                    disconnect("Camera frame watchdog expired; reconnecting");
                }
            } catch (const std::exception& error) {
                disconnect(error.what());
            }
        } else {
            control.wait(50);
        }
        control.publish(state);
    }
    state.connected = false;
    state.error = "cam_test stopped";
    control.publish(state, true);
    return 0;
}

} // namespace

auto main(int argc, char** argv) -> int {
    std::signal(SIGINT, [](int) { stop_requested = 1; });
    std::signal(SIGTERM, [](int) { stop_requested = 1; });
    try {
        const auto config = parse_config(argc, argv);
        return config ? run(*config) : 0;
    } catch (const std::exception& error) {
        log(error.what());
        return 1;
    }
}
