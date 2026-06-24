#include "module/predictor/regular/snapshot.hpp"

#include "utility/math/conversion.hpp"
#include "utility/time.hpp"

#include <memory>
#include <utility>

namespace rmcs::predictor {

struct RegularSnapshot::Impl {
    explicit Impl(EKF::XVec x, DeviceId device, CampColor color, int armor_num, TimePoint stamp)
        : x { std::move(x) }
        , device { device }
        , color { color }
        , armor_num { armor_num }
        , stamp { stamp } { }

    static auto make_armor(DeviceId device, CampColor color, int id) -> Armor3d {
        auto armor  = Armor3d {};
        armor.genre = device;
        armor.color = camp_color2armor_color(color);
        armor.id    = id;
        return armor;
    }

    static auto motion_of(EKF::XVec const& x) -> TargetMotion {
        return { Point3d { x[0], x[2], x[4] }, x[7] };
    }

    auto predict_state_at(TimePoint t) const -> EKF::XVec {
        auto const dt = util::delta_time(t, stamp).count();
        return EKFParameters::f(dt)(x);
    }

    auto predicted_armors(TimePoint t) const -> std::vector<Armor3d> {
        auto const predicted_x = predict_state_at(t);

        auto armors = std::vector<Armor3d> {};
        armors.reserve(armor_num);

        for (int id = 0; id < armor_num; ++id) {
            auto armor          = make_armor(device, color, id);
            auto const angle    = EKFParameters::armor_yaw(device, predicted_x, id);
            auto const position = EKFParameters::h_armor_xyz(device, predicted_x, id, armor_num);

            armor.translation = position;
            armor.orientation = util::euler_to_quaternion(angle, kPredictedOtherArmorPitch, 0);
            armors.emplace_back(armor);
        }

        return armors;
    }

    EKF::XVec x;
    DeviceId device;
    CampColor color;
    int armor_num;
    TimePoint stamp;
};

RegularSnapshot::RegularSnapshot(
    EKF::XVec x, DeviceId device, CampColor color, int armor_num, TimePoint stamp)
    : pimpl { std::make_unique<Impl>(std::move(x), device, color, armor_num, stamp) } { }

RegularSnapshot::RegularSnapshot(RegularSnapshot&&) noexcept                    = default;
auto RegularSnapshot::operator=(RegularSnapshot&&) noexcept -> RegularSnapshot& = default;

RegularSnapshot::~RegularSnapshot() noexcept = default;

auto RegularSnapshot::time_stamp() const -> TimePoint { return pimpl->stamp; }

auto RegularSnapshot::device_id() const -> DeviceId { return pimpl->device; }

auto RegularSnapshot::motion_at(TimePoint t) const -> TargetMotion {
    return Impl::motion_of(pimpl->predict_state_at(t));
}

auto RegularSnapshot::predicted_armors(TimePoint t) const -> std::vector<Armor3d> {
    return pimpl->predicted_armors(t);
}

} // namespace rmcs::predictor
