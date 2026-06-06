#include "visualization.hpp"

#include "module/debug/visualization/mpc_plan_visualizer.hpp"
#include "module/debug/visualization/stream_session.hpp"
#include "utility/image/image.details.hpp"
#include "utility/math/conversion.hpp"
#include "utility/rclcpp/visual/armor.hpp"
#include "utility/rclcpp/visual/arrow.hpp"
#include "utility/rclcpp/visual/transform.hpp"
#include "utility/serializable.hpp"

#include <unordered_map>

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
        util::integer_t framerate   = 80;
        util::string_t monitor_host = "localhost";
        util::string_t monitor_port = "5000";
        util::string_t stream_type  = "RTP_JEPG";

        bool enable_stream = true;
        bool drawable      = true;
        bool publishable   = true;

        static constexpr auto metas = std::tuple {
            // clang-format off
            &Config::framerate,               "framerate",
            &Config::monitor_host,            "monitor_host",
            &Config::monitor_port,            "monitor_port",
            &Config::stream_type,             "stream_type",
            &Config::enable_stream,           "enable_stream",
            &Config::drawable,                "drawable",
            &Config::publishable,             "publishable",
            // clang-format on
        };
    };

    Config config { };
    RclcppNode rclcpp { "visual" };

    debug::StreamSession session;
    SessionConfig session_config;

    debug::MpcPlanVisualizer mpc_plan;

    std::unique_ptr<visual::Arrow> aiming_direction;
    std::unique_ptr<visual::Transform> camera_transform;
    std::unordered_map<std::string, visual::Armors> armor_publishers;

    bool is_initialized  = false;
    bool size_determined = false;

    std::vector<std::unique_ptr<IDrawable>> drawables;

    auto initialize(const YAML::Node& yaml) noexcept -> NormalResult {
        auto result = config.serialize(yaml);
        if (!result.has_value()) {
            return std::unexpected { result.error() };
        }
        rclcpp.set_pub_topic_prefix("/autoaim/");

        // 用 {} 区分初始化的部分
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

        mpc_plan.initialize(rclcpp);
        aiming_direction = std::make_unique<visual::Arrow>(visual::Arrow::Config {
            .rclcpp = rclcpp,
            .name   = "aiming_direction",
            .tf     = kOdomLink,
        });
        camera_transform = std::make_unique<visual::Transform>(visual::Transform::Config {
            .rclcpp       = rclcpp,
            .topic        = "odom_to_camera_transform",
            .parent_frame = kOdomLink,
            .child_frame  = kCameraLink,
        });

        is_initialized = true;
        return { };
    }

    auto initialized() const noexcept { return is_initialized; }

    auto send_image(Image& image) noexcept -> bool {
        if (!is_initialized) return false;
        if (!config.enable_stream) return false;

        if (config.drawable) {
            auto canvas = Canvas { image };
            for (const auto& drawable : drawables)
                drawable->use(canvas);
            drawables.clear();
        }

        const auto& mat = image.details().mat;

        if (!size_determined) {
            session_config.format.w = mat.cols;
            session_config.format.h = mat.rows;

            auto open_result = session.open(session_config);
            if (!open_result) {
                rclcpp.error("Failed to open visualization session");
                rclcpp.error("  e: {}", open_result.error());
                return false;
            }
            rclcpp.info("Visualization session is opened");

            size_determined = true;
        }
        if (!session.opened()) return false;

        return session.push_frame(mat);
    }

    auto draw_later(std::unique_ptr<IDrawable> drawable) -> void {
        if (is_initialized) {
            drawables.emplace_back(std::move(drawable));
        }
    }

    auto publish(std::span<const Armor3d> armors, const std::string& name) -> void {
        if (!is_initialized) return;
        if (!config.publishable) return;

        auto iter = armor_publishers.find(name);
        if (iter == armor_publishers.end()) {
            auto config = visual::Armors::Config {
                .rclcpp = rclcpp,
                .name   = name,
                .tf     = kOdomLink,
            };
            iter = armor_publishers.emplace(name, std::move(config)).first;
        }
        iter->second.update(armors);
    }

    auto update_aiming_direction(double yaw, double pitch) const -> void {
        if (!is_initialized) return;
        if (!config.publishable) return;
        aiming_direction->move(Translation::kZero(), euler_to_quaternion(yaw, pitch, 0.0));
        aiming_direction->update();
    }

    auto update_mpc_plan(double yaw, double pitch, double yaw_rate, double pitch_rate,
        double yaw_acc, double pitch_acc) const -> void {
        if (!is_initialized) return;
        if (!config.publishable) return;
        mpc_plan.publish_planned_yaw(yaw, yaw_rate, yaw_acc);
        mpc_plan.publish_planned_pitch(pitch, pitch_rate, pitch_acc);
    }

    auto update_camera_pose(const Orientation& orientation) const -> void {
        if (!is_initialized) return;
        if (!config.publishable) return;
        camera_transform->move(Translation::kZero(), orientation);
        camera_transform->update();
    }
};

auto Visualization::initialize(const YAML::Node& yaml) noexcept
    -> std::expected<void, std::string> {
    return pimpl->initialize(yaml);
}

auto Visualization::initialized() const noexcept -> bool { return pimpl->initialized(); }

auto Visualization::update_image(Image& image) -> bool { return pimpl->send_image(image); }

auto Visualization::draw_later(std::unique_ptr<IDrawable> drawable) -> void {
    pimpl->draw_later(std::move(drawable));
}

auto Visualization::publish(std::span<const Armor3d> armors, const std::string& name) -> void {
    return pimpl->publish(armors, name);
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
