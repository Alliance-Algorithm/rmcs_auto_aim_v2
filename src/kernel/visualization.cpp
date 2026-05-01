#include "visualization.hpp"

#include <fstream>

#include "module/debug/visualization/armor_visualizer.hpp"
#include "module/debug/visualization/mpc_plan_visualizer.hpp"
#include "module/debug/visualization/stream_session.hpp"
#include "utility/image/image.details.hpp"
#include "utility/logging/printer.hpp"
#include "utility/math/conversion.hpp"
#include "utility/rclcpp/visual/arrow.hpp"
#include "utility/rclcpp/visual/transform.hpp"
#include "utility/serializable.hpp"

using namespace rmcs::kernel;
using namespace rmcs::util;

constexpr std::array kVideoTypes {
    "RTP_JEPG",
    "RTP_H264",
};

struct Visualization::Impl {
    static constexpr auto kCameraLink = "camera_link";
    static constexpr auto kOdomLink   = "odom_imu_link";

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

    Printer log { "visual" };

    std::unique_ptr<debug::StreamSession> session;
    SessionConfig session_config;

    std::unique_ptr<debug::ArmorVisualizer> armors_detect;
    std::unique_ptr<debug::ArmorVisualizer> armors_group;
    std::unique_ptr<debug::MpcPlanVisualizer> mpc_plan;
    std::unique_ptr<visual::Arrow> aiming_direction;
    std::unique_ptr<visual::Transform> camera_transform;

    bool is_initialized  = false;
    bool size_determined = false;

    Impl() noexcept {
        session       = std::make_unique<debug::StreamSession>();
        armors_detect = std::make_unique<debug::ArmorVisualizer>();
        armors_group  = std::make_unique<debug::ArmorVisualizer>();
        mpc_plan      = std::make_unique<debug::MpcPlanVisualizer>();
    }

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

        armors_detect->initialize(visual_node);
        armors_group->initialize(visual_node);
        mpc_plan->initialize(visual_node);
        aiming_direction = std::make_unique<visual::Arrow>(visual::Arrow::Config {
            .rclcpp = visual_node,
            .name   = "aiming_direction",
            .tf     = kOdomLink,
        });
        camera_transform = std::make_unique<visual::Transform>(visual::Transform::Config {
            .rclcpp       = visual_node,
            .topic        = "odom_to_camera_transform",
            .parent_frame = kOdomLink,
            .child_frame  = kCameraLink,
        });

        is_initialized = true;
        return {};
    }

    auto initialized() const noexcept { return is_initialized; }

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

    auto update_visible_armors(std::span<Armor3D const> armors) const -> bool {
        if (!is_initialized) return false;
        return armors_detect->visualize(armors, "visible_armors", kCameraLink);
    }

    auto update_visible_robot(std::span<Armor3D const> armors) const -> bool {
        if (!is_initialized) return false;
        return armors_group->visualize(armors, "visible_robot", kOdomLink);
    }

    auto update_aiming_direction(double yaw, double pitch) const -> void {
        if (!is_initialized) return;
        aiming_direction->move(Translation::kZero(), euler_to_quaternion(yaw, pitch, 0.0));
        aiming_direction->update();
    }

    auto update_mpc_plan(double yaw, double pitch, double yaw_rate, double pitch_rate,
        double yaw_acc, double pitch_acc) const -> void {
        if (!is_initialized) return;
        mpc_plan->publish_planned_yaw(yaw, yaw_rate, yaw_acc);
        mpc_plan->publish_planned_pitch(pitch, pitch_rate, pitch_acc);
    }

    auto update_camera_pose(const Orientation& orientation) const -> void {
        if (!is_initialized) return;
        camera_transform->move(Translation::kZero(), orientation);
        camera_transform->update();
    }
};

auto Visualization::initialize(const YAML::Node& yaml, RclcppNode& visual_node) noexcept
    -> std::expected<void, std::string> {
    return pimpl->initialize(yaml, visual_node);
}

auto Visualization::initialized() const noexcept -> bool { return pimpl->initialized(); }

auto Visualization::update_image(const Image& image) noexcept -> bool {
    return pimpl->send_image(image);
}

auto Visualization::update_visible_armors(std::span<Armor3D const> armors) const -> bool {
    return pimpl->update_visible_armors(armors);
}
auto Visualization::update_visible_robot(std::span<Armor3D const> armors) const -> bool {
    return pimpl->update_visible_robot(armors);
}

auto Visualization::update_aiming_direction(double yaw, double pitch) const -> void {
    pimpl->update_aiming_direction(yaw, pitch);
}

auto Visualization::update_mpc_plan(double yaw, double pitch, double yaw_rate, double pitch_rate,
    double yaw_acc, double pitch_acc) const -> void {
    pimpl->update_mpc_plan(yaw, pitch, yaw_rate, pitch_rate, yaw_acc, pitch_acc);
}

auto Visualization::update_camera_pose(const Orientation& orientation) const -> void {
    pimpl->update_camera_pose(orientation);
}

Visualization::Visualization() noexcept
    : pimpl { std::make_unique<Impl>() } { }

Visualization::~Visualization() noexcept = default;
