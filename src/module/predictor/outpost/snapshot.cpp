#include "module/predictor/outpost/snapshot.hpp"

#include "utility/math/angle.hpp"
#include "utility/time.hpp"

#include <cmath>
#include <memory>
#include <utility>

#include <eigen3/Eigen/Geometry>

namespace rmcs::predictor {

struct OutpostSnapshot::Impl {
    explicit Impl(OutpostModel::State state, std::vector<Armor3d> armors, TimePoint stamp)
        : state { state }
        , armors { std::move(armors) }
        , stamp { stamp } { }

    static auto motion_of(OutpostModel::State const& state) -> TargetMotion {
        return {
            Point3d { state.x, state.y, state.z },
            state.rotation_speed,
        };
    }

    auto predict_state_at(TimePoint t) const -> OutpostModel::State {
        auto predicted = state;
        auto const dt  = util::delta_time(t, stamp).count();
        predicted.rotation_angle =
            util::normalize_angle(state.rotation_angle + state.rotation_speed * dt);
        return predicted;
    }

    auto predicted_armors(TimePoint t) const -> std::vector<Armor3d> {
        auto const dt          = util::delta_time(t, stamp).count();
        auto const angle_delta = state.rotation_speed * dt;
        auto const center      = Eigen::Vector3d { state.x, state.y, state.z };
        auto const rotation    = Eigen::AngleAxisd { angle_delta, Eigen::Vector3d::UnitZ() };

        auto armors = std::vector<Armor3d> {};
        armors.reserve(this->armors.size());

        for (auto armor : this->armors) {
            auto position     = armor.translation.make<Eigen::Vector3d>();
            position          = center + rotation * (position - center);
            armor.translation = Translation { position };

            auto orientation = armor.orientation.make<Eigen::Quaterniond>();
            orientation = Eigen::AngleAxisd { angle_delta, Eigen::Vector3d::UnitZ() } * orientation;
            armor.orientation = Orientation { orientation.normalized() };

            armors.emplace_back(armor);
        }

        return armors;
    }

    OutpostModel::State state;
    std::vector<Armor3d> armors;
    TimePoint stamp;
};

OutpostSnapshot::OutpostSnapshot(
    OutpostModel::State state, std::vector<Armor3d> armors, TimePoint stamp)
    : pimpl { std::make_unique<Impl>(state, std::move(armors), stamp) } { }

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
