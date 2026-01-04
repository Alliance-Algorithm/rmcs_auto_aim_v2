#include "fire_control.hpp"

#include "module/fire_control/aim_point_chooser.hpp"
#include "utility/serializable.hpp"

using namespace rmcs::kernel;

struct FireControl::Impl {
    struct Config : util::Serializable {
        YAML::Node aim_point_chooser;

        constexpr static std::tuple metas {
            &Config::aim_point_chooser,
            "aim_point_chooser",
        };
    };

    fire_control::AimPointChooser aim_point_chooser;

    auto initialize(const YAML::Node& yaml) noexcept -> std::expected<void, std::string> {
        if (auto sub = yaml["aim_point_chooser"]) {
            if (auto ret = aim_point_chooser.initialize(sub); !ret.has_value()) {
                return std::unexpected { ret.error() };
            }
        }
        return {};
    }

    auto choose_armor(std::span<const Armor3D> armors, const Eigen::Vector<double, 11>& ekf_x)
        -> std::optional<Armor3D> {
        return aim_point_chooser.choose_armor(armors, ekf_x);
    }
};

FireControl::FireControl() noexcept
    : pimpl { std::make_unique<Impl>() } { }
FireControl::~FireControl() noexcept = default;

auto FireControl::initialize(const YAML::Node& yaml) noexcept -> std::expected<void, std::string> {
    return pimpl->initialize(yaml);
}

auto FireControl::choose_armor(std::span<const Armor3D> armors,
    const Eigen::Vector<double, 11>& ekf_x) -> std::optional<Armor3D> {
    return pimpl->choose_armor(armors, ekf_x);
}
