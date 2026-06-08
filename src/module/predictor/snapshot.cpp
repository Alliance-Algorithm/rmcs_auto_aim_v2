#include "snapshot.hpp"

#include "module/predictor/outpost/snapshot.hpp"
#include "module/predictor/regular/snapshot.hpp"

#include <memory>
#include <utility>
#include <variant>

namespace rmcs::predictor {

struct Snapshot::Impl {
    using SnapshotVariant = std::variant<RegularSnapshot, OutpostSnapshot>;

    explicit Impl(RegularSnapshot snapshot)
        : snapshot { std::move(snapshot) } { }

    explicit Impl(OutpostSnapshot snapshot)
        : snapshot { std::move(snapshot) } { }

    SnapshotVariant snapshot;
};

Snapshot::Snapshot(RegularSnapshot snapshot)
    : pimpl { std::make_unique<Impl>(std::move(snapshot)) } { }

Snapshot::Snapshot(OutpostSnapshot snapshot)
    : pimpl { std::make_unique<Impl>(std::move(snapshot)) } { }

Snapshot::Snapshot(Snapshot&&) noexcept                    = default;
auto Snapshot::operator=(Snapshot&&) noexcept -> Snapshot& = default;

Snapshot::~Snapshot() noexcept = default;

auto Snapshot::time_stamp() const -> TimePoint {
    return std::visit([](auto const& snapshot) { return snapshot.time_stamp(); }, pimpl->snapshot);
}

auto Snapshot::device_id() const -> DeviceId {
    return std::visit([](auto const& snapshot) { return snapshot.device_id(); }, pimpl->snapshot);
}

auto Snapshot::motion() const -> TargetMotion { return motion_at(time_stamp()); }

auto Snapshot::motion_at(TimePoint t) const -> TargetMotion {
    return std::visit([t](auto const& snapshot) { return snapshot.motion_at(t); }, pimpl->snapshot);
}

auto Snapshot::predicted_armors(TimePoint t) const -> std::vector<Armor3d> {
    return std::visit(
        [t](auto const& snapshot) { return snapshot.predicted_armors(t); }, pimpl->snapshot);
}

} // namespace rmcs::predictor
