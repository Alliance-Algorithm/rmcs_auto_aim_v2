#include "module/predictor/outpost/snapshot.hpp"

#include "utility/math/conversion.hpp"
#include "utility/time.hpp"

#include <memory>
#include <utility>

namespace rmcs::predictor {

struct OutpostSnapshot::Impl {
    explicit Impl(EKF::XVec x, CampColor color, TimePoint stamp, OutpostArmorLayout layout,
        double angular_velocity)
        : x { std::move(x) }
        , color { color }
        , stamp { stamp }
        , layout { layout }
        , angular_velocity { angular_velocity } { }

    static auto make_armor(DeviceId device, CampColor color, int id) -> Armor3d {
        auto armor  = Armor3d {};
        armor.genre = device;
        armor.color = camp_color2armor_color(color);
        armor.id    = id;
        return armor;
    }

    static auto motion_of(EKF::XVec const& x, double angular_velocity) -> TargetMotion {
        return {
            Eigen::Vector3d { x[0], x[2], x[4] },
            angular_velocity,
        };
    }

    auto predict_state_at(TimePoint t) const -> EKF::XVec {
        auto const dt = util::delta_time(t, stamp).count();
        return OutpostEKFParameters::f(dt, angular_velocity)(x);
    }

    auto predicted_armors(TimePoint t) const -> std::vector<Armor3d> {
        auto const predicted_x = predict_state_at(t);

        auto armors = std::vector<Armor3d> {};
        armors.reserve(OutpostEKFParameters::kOutpostArmorCount);

        for (int id = 0; id < OutpostEKFParameters::kOutpostArmorCount; ++id) {
            auto const& slot = layout.slots[id];
            if (!slot.assigned) continue;

            auto armor          = make_armor(DeviceId::OUTPOST, color, id);
            auto const angle    = OutpostEKFParameters::armor_yaw(predicted_x, layout, id);
            auto const position = OutpostEKFParameters::h_armor_xyz(
                predicted_x, slot.phase_offset, slot.height_offset);

            armor.translation = position;
            armor.orientation = util::euler_to_quaternion(angle, kPredictedOutpostArmorPitch, 0);
            armors.emplace_back(armor);
        }

        return armors;
    }

    EKF::XVec x;
    CampColor color;
    TimePoint stamp;
    OutpostArmorLayout layout;
    double angular_velocity { 0.0 };
};

OutpostSnapshot::OutpostSnapshot(EKF::XVec x, CampColor color, TimePoint stamp,
    OutpostArmorLayout layout, double angular_velocity)
    : pimpl { std::make_unique<Impl>(std::move(x), color, stamp, layout, angular_velocity) } { }

OutpostSnapshot::OutpostSnapshot(OutpostSnapshot&&) noexcept                    = default;
auto OutpostSnapshot::operator=(OutpostSnapshot&&) noexcept -> OutpostSnapshot& = default;

OutpostSnapshot::~OutpostSnapshot() noexcept = default;

auto OutpostSnapshot::time_stamp() const -> TimePoint { return pimpl->stamp; }

auto OutpostSnapshot::device_id() const -> DeviceId { return DeviceId::OUTPOST; }

auto OutpostSnapshot::motion_at(TimePoint t) const -> TargetMotion {
    return Impl::motion_of(pimpl->predict_state_at(t), pimpl->angular_velocity);
}

auto OutpostSnapshot::predicted_armors(TimePoint t) const -> std::vector<Armor3d> {
    return pimpl->predicted_armors(t);
}

} // namespace rmcs::predictor
