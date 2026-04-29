#pragma once

#include <eigen3/Eigen/Core>
#include <memory>
#include <vector>

#include "utility/clock.hpp"
#include "utility/math/kalman_filter/ekf.hpp"
#include "utility/robot/id.hpp"
#include "utility/robot/armor.hpp"

namespace rmcs::predictor {

struct ISnapshotBackend;
class Snapshot;

namespace detail {
    auto make_snapshot(std::unique_ptr<ISnapshotBackend> backend) noexcept -> Snapshot;
}

class Snapshot {
public:
    using NormalEKF  = util::EKF<11, 4>;
    using OutpostEKF = util::EKF<6, 4>;

    struct Kinematics {
        Eigen::Vector3d center_position;
        double angular_velocity;
    };

    static auto empty(TimePoint stamp) noexcept -> Snapshot;

    Snapshot(Snapshot const&) = delete;
    Snapshot(Snapshot&&) noexcept;
    Snapshot& operator=(Snapshot const&) = delete;
    Snapshot& operator=(Snapshot&&) noexcept;
    ~Snapshot() noexcept;

    auto time_stamp() const -> TimePoint;
    auto device_id() const -> DeviceId;
    auto kinematics() const -> Kinematics;
    auto kinematics_at(TimePoint t) const -> Kinematics;

    auto predicted_armors() const { return predicted_armors(Clock::now()); }

    auto predicted_armors(TimePoint t) const -> std::vector<Armor3D>;

private:
    explicit Snapshot(std::unique_ptr<ISnapshotBackend> backend) noexcept;

    std::unique_ptr<ISnapshotBackend> backend;

    friend auto detail::make_snapshot(std::unique_ptr<ISnapshotBackend> backend) noexcept
        -> Snapshot;
};

}
