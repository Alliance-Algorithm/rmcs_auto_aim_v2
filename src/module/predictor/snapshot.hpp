#pragma once

#include "utility/clock.hpp"
#include "utility/robot/armor.hpp"
#include "utility/robot/id.hpp"

#include <eigen3/Eigen/Core>

#include <memory>
#include <vector>

namespace rmcs::predictor {

class RegularSnapshot;
class OutpostSnapshot;
class Snapshot;

struct TargetMotion {
    Eigen::Vector3d center_position;
    double angular_velocity;
};

class Snapshot {
public:
    explicit Snapshot(RegularSnapshot snapshot);
    explicit Snapshot(OutpostSnapshot snapshot);
    Snapshot(Snapshot const&) = delete;
    Snapshot(Snapshot&&) noexcept;
    Snapshot& operator=(Snapshot const&) = delete;
    Snapshot& operator=(Snapshot&&) noexcept;
    ~Snapshot() noexcept;

    auto time_stamp() const -> TimePoint;
    auto device_id() const -> DeviceId;

    auto motion() const -> TargetMotion;
    auto motion_at(TimePoint t) const -> TargetMotion;

    auto predicted_armors() const { return predicted_armors(Clock::now()); }
    auto predicted_armors(TimePoint t) const -> std::vector<Armor3d>;

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl;
};

} // namespace rmcs::predictor
