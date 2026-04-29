#include "module/fire_control/strategy/aim_point_provider.hpp"

#include <cmath>
#include <memory>

#include "module/fire_control/aim_point_chooser.hpp"
#include "utility/math/angle.hpp"
#include "utility/serializable.hpp"

using namespace rmcs::fire_control;

struct AimPointProvider::Impl {
    AimPointChooser chooser {};

    struct Config : rmcs::util::Serializable {
        double coming_angle;
        double leaving_angle;
        double outpost_coming_angle;
        double outpost_leaving_angle;

        constexpr static std::tuple metas {
            &Config::coming_angle,
            "coming_angle",
            &Config::leaving_angle,
            "leaving_angle",
            &Config::outpost_coming_angle,
            "outpost_coming_angle",
            &Config::outpost_leaving_angle,
            "outpost_leaving_angle",
        };
    } config;

    Impl() noexcept = default;

    auto aim_point_at(predictor::Snapshot const& snapshot, TimePoint t, Mode mode)
        -> std::optional<Eigen::Vector3d> {
        if (mode == Mode::CENTER) return aim_point_from_center(snapshot, t);
        return aim_point_from_armor(snapshot, t);
    }

    auto configure_yaml(const YAML::Node& yaml) noexcept -> std::expected<void, std::string> {
        auto result = config.serialize(yaml);
        if (!result.has_value()) {
            return std::unexpected { result.error() };
        }

        auto chooser_config = AimPointChooser::Config {
            .coming_angle          = util::deg2rad(config.coming_angle),
            .leaving_angle         = util::deg2rad(config.leaving_angle),
            .outpost_coming_angle  = util::deg2rad(config.outpost_coming_angle),
            .outpost_leaving_angle = util::deg2rad(config.outpost_leaving_angle),
        };
        chooser.initialize(chooser_config);
        return {};
    }

    auto aim_point_from_armor(predictor::Snapshot const& snapshot, TimePoint t)
        -> std::optional<Eigen::Vector3d> {
        auto predicted_armors     = snapshot.predicted_armors(t);
        auto predicted_kinematics = snapshot.kinematics_at(t);

        auto chosen = chooser.choose_armor(predicted_armors, predicted_kinematics.center_position,
            predicted_kinematics.angular_velocity);
        if (!chosen) return std::nullopt;

        auto aim_position = Eigen::Vector3d {};
        chosen->translation.copy_to(aim_position);
        return aim_position;
    }

    static auto aim_point_from_center(predictor::Snapshot const& snapshot, TimePoint t)
        -> std::optional<Eigen::Vector3d> {
        auto target_kinematics = snapshot.kinematics_at(t);

        auto const target_d = std::hypot(
            target_kinematics.center_position.x(), target_kinematics.center_position.y());
        if (!(target_d > 0.0)) return std::nullopt;

        return target_kinematics.center_position;
    }
};

AimPointProvider::AimPointProvider() noexcept
    : pimpl { std::make_unique<Impl>() } { }

AimPointProvider::~AimPointProvider() noexcept = default;

auto AimPointProvider::configure_yaml(const YAML::Node& yaml) noexcept
    -> std::expected<void, std::string> {
    return pimpl->configure_yaml(yaml);
}

auto AimPointProvider::aim_point_at(predictor::Snapshot const& snapshot, TimePoint t, Mode mode)
    -> std::optional<Eigen::Vector3d> {
    return pimpl->aim_point_at(snapshot, t, mode);
}
