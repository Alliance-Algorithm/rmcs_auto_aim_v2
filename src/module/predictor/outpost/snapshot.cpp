#include "module/predictor/outpost/armor_layout.hpp"
#include "module/predictor/snapshot.hpp"
#include "utility/robot/color.hpp"

#include <algorithm>
#include <utility>

#include "module/predictor/backend/snapshot_backend.hpp"
#include "module/predictor/outpost/ekf_parameter.hpp"
#include "utility/math/conversion.hpp"
#include "utility/time.hpp"

namespace rmcs::predictor {

namespace {
    auto make_armor(DeviceId device, CampColor color, int id) -> Armor3D {
        auto armor  = Armor3D {};
        armor.genre = device;
        armor.color = camp_color2armor_color(color);
        armor.id    = id;
        return armor;
    }

    struct OutpostSnapshotBackend final : ISnapshotBackend {
        explicit OutpostSnapshotBackend(Snapshot::OutpostEKF::XVec x, CampColor color,
            int armor_num, TimePoint stamp, OutpostArmorLayout layout) noexcept
            : ISnapshotBackend { DeviceId::OUTPOST, color, armor_num, stamp }
            , x { std::move(x) }
            , layout { layout } { }

        [[nodiscard]] auto kinematics_at(TimePoint t) const -> Snapshot::Kinematics override {
            return kinematics_of(predict_state_at(t));
        }

        [[nodiscard]] auto predicted_armors(TimePoint t) const -> std::vector<Armor3D> override {
            auto const predicted_x = predict_state_at(t);

            auto armors = std::vector<Armor3D> {};
            armors.reserve(std::clamp(armor_num, 0, OutpostEKFParameters::kOutpostArmorCount));

            for (int id = 0; id < OutpostEKFParameters::kOutpostArmorCount; ++id) {
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
            double height_sum  = 0.0;
            int assigned_count = 0;
            for (int id = 0; id < OutpostEKFParameters::kOutpostArmorCount; ++id) {
                if (!layout.slots[id].assigned) continue;
                height_sum += layout.slots[id].height_offset;
                assigned_count++;
            }

            auto const center_z = x[4]
                + (assigned_count == 0 ? 0.0 : height_sum / static_cast<double>(assigned_count));
            return { Eigen::Vector3d { x[0], x[2], center_z }, x[6] };
        }

        auto predict_state_at(TimePoint t) const -> Snapshot::OutpostEKF::XVec {
            auto const dt = util::delta_time(t, stamp).count();
            return OutpostEKFParameters::f(dt)(x);
        }

        Snapshot::OutpostEKF::XVec x;
        OutpostArmorLayout layout;
    };

} // namespace

auto make_outpost_snapshot(Snapshot::OutpostEKF::XVec ekf_x, CampColor color, int armor_num,
    TimePoint stamp, OutpostArmorLayout outpost_layout) noexcept -> Snapshot {
    return Snapshot { std::make_unique<OutpostSnapshotBackend>(
        std::move(ekf_x), color, armor_num, stamp, outpost_layout) };
}

} // namespace rmcs::predictor
