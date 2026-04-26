#pragma once

#include <cstdint>
#include <memory>
#include <span>

#include "module/predictor/snapshot.hpp"

namespace rmcs::predictor {

enum class RobotStateBackendKind : std::uint8_t {
    Regular,
    Outpost,
};

class IRobotStateBackend {
public:
    virtual ~IRobotStateBackend() noexcept = default;

    virtual auto initialize(Armor3D const& armor, TimePoint t) -> void = 0;
    virtual auto predict(TimePoint t) -> void                          = 0;

    virtual auto update(std::span<Armor3D const> armors) -> bool = 0;

    [[nodiscard]] virtual auto is_converged() const -> bool     = 0;
    [[nodiscard]] virtual auto get_snapshot() const -> Snapshot = 0;
    [[nodiscard]] virtual auto distance() const -> double       = 0;
};

[[nodiscard]] constexpr auto classify_robot_state_backend(DeviceId device) noexcept
    -> RobotStateBackendKind {
    switch (device) {
    case DeviceId::OUTPOST:
        return RobotStateBackendKind::Outpost;
    default:
        return RobotStateBackendKind::Regular;
    }
}

[[nodiscard]] auto make_robot_state_backend(RobotStateBackendKind kind, TimePoint stamp)
    -> std::unique_ptr<IRobotStateBackend>;

} // namespace rmcs::predictor
