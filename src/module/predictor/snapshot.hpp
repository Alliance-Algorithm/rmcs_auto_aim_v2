#pragma once

#include <memory>
#include <vector>

#include "utility/clock.hpp"
#include "utility/robot/armor.hpp"
#include "utility/robot/id.hpp"

namespace rmcs::predictor {

struct ISnapshotBackend;
class Snapshot;

namespace detail {
    auto make_snapshot(std::unique_ptr<ISnapshotBackend> backend) noexcept -> Snapshot;
}

class Snapshot {
public:
    struct Kinematics {
        Point3d center_position;
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

    auto predicted_armors(TimePoint t) const -> std::vector<Armor3d>;

private:
    explicit Snapshot(std::unique_ptr<ISnapshotBackend> backend) noexcept;

    std::unique_ptr<ISnapshotBackend> backend;

    friend auto detail::make_snapshot(std::unique_ptr<ISnapshotBackend> backend) noexcept
        -> Snapshot;
};

}
