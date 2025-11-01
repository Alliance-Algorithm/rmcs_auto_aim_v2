#include "visualization.hpp"
#include "modules/debug/visualization/stream_session.hpp"
#include "utility/image.details.hpp"
#include "utility/logging/printer.hpp"

using do_not_warn_please_for_clangd = rmcs::Image::Details;
using namespace rmcs::kernel;

struct VisualizationRuntime::Impl {
    using SessionConfig = debug::StreamSession::Config;
    using NormalResult  = std::expected<void, std::string>;

    Printer log { "visualization" };

    std::unique_ptr<debug::StreamSession> session;
    SessionConfig session_config;

    bool has_determine_size = false;

    Impl() noexcept { session = std::make_unique<debug::StreamSession>(); }

    auto initialize(const Config& config) noexcept -> NormalResult {
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
        return {};
    }
    auto send_image(const Image& image) noexcept -> bool {
        const auto& mat = image.details().get_mat();
        if (has_determine_size == false) {
            has_determine_size = true;

            session_config.format.w = mat.cols;
            session_config.format.h = mat.rows;

            if (auto ret = session->open(session_config)) {
                log.info("Visualization session is opened");
                if (auto ret = session->session_description_protocol()) {
                    log.info("\n{}", ret.value());
                } else {
                    log.error("Failed to get description protocol");
                    log.error("  e: {}", ret.error());
                }
            } else {
                log.error("Failed to open visualization session");
                log.error("  e: {}", ret.error());
            }
        }
        if (!session->opened()) return false;

        return session->push_frame(mat);
    }
};

auto VisualizationRuntime::initialize(const Config& config) noexcept
    -> std::expected<void, std::string> {
    return pimpl->initialize(config);
}

auto VisualizationRuntime::send_image(const Image& image) noexcept -> bool {
    return pimpl->send_image(image);
}

VisualizationRuntime::VisualizationRuntime() noexcept
    : pimpl { std::make_unique<Impl>() } { }

VisualizationRuntime::~VisualizationRuntime() noexcept = default;
