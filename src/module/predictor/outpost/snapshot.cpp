#include "module/predictor/outpost/snapshot.hpp"

#include "utility/math/conversion.hpp"
#include "utility/time.hpp"

#include <memory>
#include <utility>

namespace rmcs::predictor {

struct OutpostSnapshot::Impl {
    explicit Impl(EKF::XVec x, CampColor color, TimePoint stamp, OutpostArmorLayout layout)
        : x { std::move(x) }
        , color { color }
        , stamp { stamp }
        , layout { layout } { }

    static auto make_armor(DeviceId device, CampColor color, int id) -> Armor3d {
        auto armor  = Armor3d {};
        armor.genre = device;
        armor.color = camp_color2armor_color(color);
        armor.id    = id;
        return armor;
    }

    static auto motion_of(EKF::XVec const& x) -> TargetMotion {
        return { Eigen::Vector3d { x[0], x[2], x[4] }, x[6] };
    }

    auto predict_state_at(TimePoint t) const -> EKF::XVec {
        auto const dt = util::delta_time(t, stamp).count();
        return OutpostEKFParameters::f(dt)(x);
    }

    auto predicted_armors(TimePoint t) const -> std::vector<Armor3d> {
        auto const predicted_x = predict_state_at(t);
        auto const predicted_armor_count =
            layout.height_template.has_value() ? OutpostEKFParameters::kOutpostArmorCount : 1;

        auto armors = std::vector<Armor3d> {};
        armors.reserve(predicted_armor_count);

        for (int id = 0; id < predicted_armor_count; ++id) {
            auto armor          = make_armor(DeviceId::OUTPOST, color, id);
            auto const height   = OutpostEKFParameters::height_offset(layout, armor.id);
            auto const angle    = OutpostEKFParameters::armor_yaw(predicted_x, id);
            auto const position = OutpostEKFParameters::h_armor_xyz(
                predicted_x, OutpostEKFParameters::phase_offset(armor.id), height);

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
};

OutpostSnapshot::OutpostSnapshot(
    EKF::XVec x, CampColor color, TimePoint stamp, OutpostArmorLayout layout)
    : pimpl { std::make_unique<Impl>(std::move(x), color, stamp, layout) } { }

OutpostSnapshot::OutpostSnapshot(OutpostSnapshot&&) noexcept                    = default;
auto OutpostSnapshot::operator=(OutpostSnapshot&&) noexcept -> OutpostSnapshot& = default;

OutpostSnapshot::~OutpostSnapshot() noexcept = default;

auto OutpostSnapshot::time_stamp() const -> TimePoint { return pimpl->stamp; }

auto OutpostSnapshot::device_id() const -> DeviceId { return DeviceId::OUTPOST; }

auto OutpostSnapshot::motion_at(TimePoint t) const -> TargetMotion {
    return Impl::motion_of(pimpl->predict_state_at(t));
}

auto OutpostSnapshot::predicted_armors(TimePoint t) const -> std::vector<Armor3d> {
    return pimpl->predicted_armors(t);
}

} // namespace rmcs::predictor
