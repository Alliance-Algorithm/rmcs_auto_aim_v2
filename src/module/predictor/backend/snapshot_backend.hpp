#pragma once

#include "module/predictor/snapshot.hpp"
#include "utility/robot/color.hpp"
#include "utility/robot/id.hpp"

namespace rmcs::predictor {

struct ISnapshotBackend {
    DeviceId device;
    CampColor color;
    int armor_num;
    TimePoint stamp;

    ISnapshotBackend(DeviceId device, CampColor color, int armor_num, TimePoint stamp) noexcept
        : device { device }
        , color { color }
        , armor_num { armor_num }
        , stamp { stamp } { }

    virtual ~ISnapshotBackend() noexcept = default;

    [[nodiscard]] virtual auto kinematics_at(TimePoint t) const -> Snapshot::Kinematics    = 0;
    [[nodiscard]] virtual auto predicted_armors(TimePoint t) const -> std::vector<Armor3D> = 0;

    auto time_stamp() const -> TimePoint { return stamp; }
};

} // namespace rmcs::predictor
