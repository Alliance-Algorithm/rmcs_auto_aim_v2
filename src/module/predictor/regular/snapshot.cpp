#include "module/predictor/regular/snapshot.hpp"

#include <utility>

#include "module/predictor/backend/snapshot_backend.hpp"
#include "module/predictor/regular/ekf_parameter.hpp"
#include "utility/math/conversion.hpp"
#include "utility/time.hpp"

namespace rmcs::predictor {

namespace {

    auto make_armor(DeviceId device, CampColor color, int id) -> Armor3D {
        auto armor  = Armor3D { };
        armor.genre = device;
        armor.color = camp_color2armor_color(color);
        armor.id    = id;
        return armor;
    }

    struct RegularSnapshotBackend final : ISnapshotBackend {
        explicit RegularSnapshotBackend(Snapshot::NormalEKF::XVec x, DeviceId device,
            CampColor color, int armor_num, TimePoint stamp) noexcept
            : ISnapshotBackend { device, color, armor_num, stamp }
            , x { std::move(x) } { }

        [[nodiscard]] auto kinematics_at(TimePoint t) const -> Snapshot::Kinematics override {
            return kinematics_of(predict_state_at(t));
        }

        [[nodiscard]] auto predicted_armors(TimePoint t) const -> std::vector<Armor3D> override {
            auto const predicted_x = predict_state_at(t);

            auto armors = std::vector<Armor3D> { };
            armors.reserve(armor_num);

            for (int id = 0; id < armor_num; ++id) {
                auto armor       = make_armor(device, color, id);
                auto const angle = EKFParameters::armor_yaw(device, predicted_x, id);
                auto const position =
                    EKFParameters::h_armor_xyz(device, predicted_x, id, armor_num);

                armor.translation = position;
                armor.orientation = util::euler_to_quaternion(angle, kPredictedOtherArmorPitch, 0);
                armors.emplace_back(armor);
            }

            return armors;
        }

    private:
        static auto kinematics_of(Snapshot::NormalEKF::XVec const& x) -> Snapshot::Kinematics {
            return { Eigen::Vector3d { x[0], x[2], x[4] }, x[7] };
        }

        auto predict_state_at(TimePoint t) const -> Snapshot::NormalEKF::XVec {
            auto const dt = util::delta_time(t, stamp).count();
            return EKFParameters::f(dt)(x);
        }

        Snapshot::NormalEKF::XVec x;
    };

} // namespace

auto detail::make_regular_snapshot(Snapshot::NormalEKF::XVec ekf_x, DeviceId device,
    CampColor color, int armor_num, TimePoint stamp) noexcept -> Snapshot {
    return detail::make_snapshot(std::make_unique<RegularSnapshotBackend>(
        std::move(ekf_x), device, color, armor_num, stamp));
}

} // namespace rmcs::predictor
