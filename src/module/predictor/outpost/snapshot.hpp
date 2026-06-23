#pragma once

#include "module/predictor/model/outpost.hpp"
#include "module/predictor/snapshot.hpp"

#include <memory>
#include <vector>

namespace rmcs::predictor {

class OutpostSnapshot {
public:
    explicit OutpostSnapshot(
        OutpostModel::State state, std::vector<Armor3d> armors, TimePoint stamp);
    OutpostSnapshot(OutpostSnapshot const&) = delete;
    OutpostSnapshot(OutpostSnapshot&&) noexcept;
    OutpostSnapshot& operator=(OutpostSnapshot const&) = delete;
    OutpostSnapshot& operator=(OutpostSnapshot&&) noexcept;
    ~OutpostSnapshot() noexcept;

    auto time_stamp() const -> TimePoint;
    auto device_id() const -> DeviceId;
    auto motion_at(TimePoint t) const -> TargetMotion;
    auto predicted_armors(TimePoint t) const -> std::vector<Armor3d>;

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl;
};

} // namespace rmcs::predictor
