#include "module/predictor/outpost/snapshot.hpp"

#include <algorithm>
#include <utility>

#include "module/predictor/backend/snapshot_backend.hpp"
#include "module/predictor/outpost/ekf_parameter.hpp"
#include "utility/math/conversion.hpp"
#include "utility/time.hpp"

namespace rmcs::predictor {

namespace {

    auto normalize_spin_sign(int spin_sign) -> int {
        if (spin_sign > 0) return +1;
        if (spin_sign < 0) return -1;
        return 0;
    }

    auto make_armor(DeviceId device, CampColor color, int id) -> Armor3D {
        auto armor  = Armor3D { };
        armor.genre = device;
        armor.color = camp_color2armor_color(color);
        armor.id    = id;
        return armor;
    }

    struct OutpostSnapshotBackend final : ISnapshotBackend {
        explicit OutpostSnapshotBackend(Snapshot::OutpostEKF::XVec x, CampColor color,
            int armor_num, TimePoint stamp, int spin_sign, OutpostArmorLayout layout) noexcept
            : ISnapshotBackend { DeviceId::OUTPOST, color, armor_num, stamp }
            , x { std::move(x) }
            , spin_sign { normalize_spin_sign(spin_sign) }
            , layout { layout } { }

        [[nodiscard]] auto kinematics_at(TimePoint t) const -> Snapshot::Kinematics override {
            return kinematics_of(predict_state_at(t));
        }

        [[nodiscard]] auto predicted_armors(TimePoint t) const -> std::vector<Armor3D> override {
            auto const predicted_x = predict_state_at(t);
            auto const max_armors =
                std::clamp(armor_num, 0, OutpostEKFParameters::kOutpostArmorCount);

            auto armors = std::vector<Armor3D> { };
            armors.reserve(max_armors);

            for (int id = 0; id < max_armors; ++id) {
                if (!layout.slots[id].assigned) continue;

                auto armor          = make_armor(device, color, id);
                auto const angle    = OutpostEKFParameters::armor_yaw(predicted_x, layout, id);
                auto const position = OutpostEKFParameters::h_armor_xyz(predicted_x, layout, id);

                armor.translation = position;
                armor.orientation =
                    util::euler_to_quaternion(angle, kPredictedOutpostArmorPitch, 0);
                armors.emplace_back(armor);
            }

            return armors;
        }

    private:
        auto kinematics_of(Snapshot::OutpostEKF::XVec const& x) const -> Snapshot::Kinematics {
            auto const max_armors =
                std::clamp(armor_num, 0, OutpostEKFParameters::kOutpostArmorCount);
            double height_sum  = 0.0;
            int assigned_count = 0;
            for (int id = 0; id < max_armors; ++id) {
                if (!layout.slots[id].assigned) continue;
                height_sum += layout.slots[id].height_offset;
                assigned_count++;
            }

            auto const center_z = x[4]
                + (assigned_count == 0 ? 0.0 : height_sum / static_cast<double>(assigned_count));
            auto const angular_velocity = static_cast<double>(spin_sign) * kOutpostAngularSpeed;
            return { Eigen::Vector3d { x[0], x[2], center_z }, angular_velocity };
        }

        auto predict_state_at(TimePoint t) const -> Snapshot::OutpostEKF::XVec {
            auto const dt = util::delta_time(t, stamp).count();
            return OutpostEKFParameters::f(dt, spin_sign)(x);
        }

        Snapshot::OutpostEKF::XVec x;
        int spin_sign;
        OutpostArmorLayout layout;
    };

} // namespace

auto detail::make_outpost_snapshot(Snapshot::OutpostEKF::XVec ekf_x, CampColor color, int armor_num,
    TimePoint stamp, int outpost_spin_sign, OutpostArmorLayout outpost_layout) noexcept
    -> Snapshot {
    return detail::make_snapshot(std::make_unique<OutpostSnapshotBackend>(
        std::move(ekf_x), color, armor_num, stamp, outpost_spin_sign, outpost_layout));
}

} // namespace rmcs::predictor
