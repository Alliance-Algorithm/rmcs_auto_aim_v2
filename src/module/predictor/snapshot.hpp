#pragma once

#include <eigen3/Eigen/Core>
#include <memory>
#include <vector>

#include "utility/clock.hpp"
#include "utility/math/kalman_filter/ekf.hpp"
#include "utility/robot/armor.hpp"
#include "utility/robot/id.hpp"

namespace rmcs::predictor {

struct ISnapshotBackend;
struct OutpostArmorLayout;
class Snapshot;

class Snapshot {
public:
    using NormalEKF  = util::EKF<11, 4>;
    using OutpostEKF = util::EKF<7, 4>;

    struct Kinematics {
        Eigen::Vector3d center_position;
        double angular_velocity;
    };

    explicit Snapshot(std::unique_ptr<ISnapshotBackend> backend) noexcept;
    Snapshot(Snapshot const&) = delete;
    Snapshot(Snapshot&&) noexcept;
    Snapshot& operator=(Snapshot const&) = delete;
    Snapshot& operator=(Snapshot&&) noexcept;
    ~Snapshot() noexcept;
    static auto empty(TimePoint stamp) noexcept -> Snapshot;

    static auto make_outpost(OutpostEKF::XVec ekf_x, CampColor color, TimePoint stamp,
        OutpostArmorLayout const& outpost_layout) noexcept -> Snapshot;
    static auto make_regular(NormalEKF::XVec ekf_x, DeviceId device, CampColor color, int armor_num,
        TimePoint stamp) noexcept -> Snapshot;

    auto time_stamp() const -> TimePoint;
    auto device_id() const -> DeviceId;

    auto kinematics() const -> Kinematics;
    auto kinematics_at(TimePoint t) const -> Kinematics;

    auto predicted_armors() const { return predicted_armors(Clock::now()); }
    auto predicted_armors(TimePoint t) const -> std::vector<Armor3d>;

private:
    std::unique_ptr<ISnapshotBackend> backend;
};

}
