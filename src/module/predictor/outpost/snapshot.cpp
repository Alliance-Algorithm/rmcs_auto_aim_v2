#include "module/predictor/snapshot.hpp"
#include "module/predictor/backend/snapshot_backend.hpp"
#include "module/predictor/outpost/armor_layout.hpp"
#include "module/predictor/outpost/ekf_parameter.hpp"
#include "utility/math/conversion.hpp"
#include "utility/robot/color.hpp"
#include "utility/time.hpp"

#include <algorithm>
#include <utility>

namespace rmcs::predictor {

struct OutpostSnapshotBackend final : ISnapshotBackend {
    explicit OutpostSnapshotBackend(Snapshot::OutpostEKF::XVec x, CampColor color, TimePoint stamp,
        OutpostArmorLayout layout) noexcept
        : ISnapshotBackend { DeviceId::OUTPOST, color, OutpostEKFParameters::kOutpostArmorCount,
            stamp }
        , x { std::move(x) }
        , layout { layout } { }

    [[nodiscard]] auto kinematics_at(TimePoint t) const -> Snapshot::Kinematics override {
        return kinematics_of(predict_state_at(t));
    }

    [[nodiscard]] auto predicted_armors(TimePoint t) const -> std::vector<Armor3d> override {
        auto const predicted_x = predict_state_at(t);
        auto const predicted_armor_count =
            layout.height_template.has_value() ? OutpostEKFParameters::kOutpostArmorCount : 1;

        auto armors = std::vector<Armor3d> {};
        armors.reserve(
            std::clamp(predicted_armor_count, 0, OutpostEKFParameters::kOutpostArmorCount));

        for (int id = 0; id < predicted_armor_count; ++id) {
            auto armor          = make_armor(device, color, id);
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

private:
    static auto make_armor(DeviceId device, CampColor color, int id) -> Armor3d {
        auto armor  = Armor3d {};
        armor.genre = device;
        armor.color = camp_color2armor_color(color);
        armor.id    = id;
        return armor;
    }

    static auto kinematics_of(Snapshot::OutpostEKF::XVec const& x) -> Snapshot::Kinematics {
        return { Eigen::Vector3d { x[0], x[2], x[4] }, x[6] };
    }

    auto predict_state_at(TimePoint t) const -> Snapshot::OutpostEKF::XVec {
        auto const dt = util::delta_time(t, stamp).count();
        return OutpostEKFParameters::f(dt)(x);
    }

    Snapshot::OutpostEKF::XVec x;
    OutpostArmorLayout layout;
};

auto Snapshot::make_outpost(OutpostEKF::XVec ekf_x, CampColor color, TimePoint stamp,
    OutpostArmorLayout const& outpost_layout) noexcept -> Snapshot {
    return Snapshot { std::make_unique<OutpostSnapshotBackend>(
        std::move(ekf_x), color, stamp, outpost_layout) };
}

} // namespace rmcs::predictor
