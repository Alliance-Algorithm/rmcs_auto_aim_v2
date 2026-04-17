#include "snapshot.hpp"

#include <memory>
#include <utility>

#include "module/predictor/backend/snapshot_backend.hpp"

namespace rmcs::predictor {

namespace {

struct EmptySnapshotBackend final : ISnapshotBackend {
    explicit EmptySnapshotBackend(Snapshot::Clock::time_point stamp) noexcept
        : ISnapshotBackend { DeviceId::UNKNOWN, CampColor::UNKNOWN, 0, stamp } { }

    [[nodiscard]] auto kinematics_at(Snapshot::Clock::time_point) const
        -> Snapshot::Kinematics override {
        return { Eigen::Vector3d::Zero(), 0.0 };
    }

    [[nodiscard]] auto predicted_armors(Snapshot::Clock::time_point) const
        -> std::vector<Armor3D> override {
        return {};
    }
};

} // namespace

auto detail::make_snapshot(std::unique_ptr<ISnapshotBackend> backend) noexcept -> Snapshot {
    return Snapshot { std::move(backend) };
}

auto Snapshot::empty(Clock::time_point stamp) noexcept -> Snapshot {
    return detail::make_snapshot(std::make_unique<EmptySnapshotBackend>(stamp));
}

Snapshot::Snapshot(std::unique_ptr<ISnapshotBackend> backend) noexcept
    : backend { std::move(backend) } { }

Snapshot::Snapshot(Snapshot&&) noexcept = default;
auto Snapshot::operator=(Snapshot&&) noexcept -> Snapshot& = default;

Snapshot::~Snapshot() noexcept = default;

auto Snapshot::time_stamp() const -> Clock::time_point { return backend->time_stamp(); }

auto Snapshot::kinematics() const -> Kinematics { return backend->kinematics_at(time_stamp()); }

auto Snapshot::kinematics_at(Clock::time_point t) const -> Kinematics {
    return backend->kinematics_at(t);
}

auto Snapshot::predicted_armors(Clock::time_point t) const -> std::vector<Armor3D> {
    return backend->predicted_armors(t);
}

} // namespace rmcs::predictor
