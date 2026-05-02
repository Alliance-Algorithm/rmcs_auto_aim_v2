#include "snapshot.hpp"

#include <memory>
#include <utility>

#include "module/predictor/backend/snapshot_backend.hpp"

namespace rmcs::predictor {

namespace {

    struct EmptySnapshotBackend final : ISnapshotBackend {
        explicit EmptySnapshotBackend(TimePoint stamp) noexcept
            : ISnapshotBackend { DeviceId::UNKNOWN, CampColor::UNKNOWN, 0, stamp } { }

        [[nodiscard]] auto kinematics_at(TimePoint) const -> Snapshot::Kinematics override {
            return { Eigen::Vector3d::Zero(), 0.0 };
        }

        [[nodiscard]] auto predicted_armors(TimePoint) const -> std::vector<Armor3D> override {
            return { };
        }
    };

} // namespace

auto detail::make_snapshot(std::unique_ptr<ISnapshotBackend> backend) noexcept -> Snapshot {
    return Snapshot { std::move(backend) };
}

auto Snapshot::empty(TimePoint stamp) noexcept -> Snapshot {
    return detail::make_snapshot(std::make_unique<EmptySnapshotBackend>(stamp));
}

Snapshot::Snapshot(std::unique_ptr<ISnapshotBackend> backend) noexcept
    : backend { std::move(backend) } { }

Snapshot::Snapshot(Snapshot&&) noexcept                    = default;
auto Snapshot::operator=(Snapshot&&) noexcept -> Snapshot& = default;

Snapshot::~Snapshot() noexcept = default;

auto Snapshot::time_stamp() const -> TimePoint { return backend->time_stamp(); }

auto Snapshot::device_id() const -> DeviceId { return backend->device; }

auto Snapshot::kinematics() const -> Kinematics { return backend->kinematics_at(time_stamp()); }

auto Snapshot::kinematics_at(TimePoint t) const -> Kinematics { return backend->kinematics_at(t); }

auto Snapshot::predicted_armors(TimePoint t) const -> std::vector<Armor3D> {
    return backend->predicted_armors(t);
}

} // namespace rmcs::predictor
