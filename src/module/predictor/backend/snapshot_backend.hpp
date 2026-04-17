#pragma once

#include "module/predictor/snapshot.hpp"
#include "utility/robot/color.hpp"
#include "utility/robot/id.hpp"

namespace rmcs::predictor {

struct ISnapshotBackend {
    DeviceId device;
    CampColor color;
    int armor_num;
    Snapshot::Clock::time_point stamp;

    ISnapshotBackend(DeviceId device, CampColor color, int armor_num,
        Snapshot::Clock::time_point stamp) noexcept
        : device { device }
        , color { color }
        , armor_num { armor_num }
        , stamp { stamp } { }

    virtual ~ISnapshotBackend() noexcept = default;

    [[nodiscard]] virtual auto kinematics_at(Snapshot::Clock::time_point t) const
        -> Snapshot::Kinematics = 0;
    [[nodiscard]] virtual auto predicted_armors(Snapshot::Clock::time_point t) const
        -> std::vector<Armor3D> = 0;

    auto time_stamp() const -> Snapshot::Clock::time_point { return stamp; }
};

} // namespace rmcs::predictor
