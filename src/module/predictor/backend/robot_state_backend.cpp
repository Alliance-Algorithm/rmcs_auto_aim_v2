#include "module/predictor/backend/robot_state_backend.hpp"

#include <memory>
#include <utility>

#include "module/predictor/outpost/robot_state.hpp"
#include "module/predictor/regular/robot_state.hpp"

namespace rmcs::predictor {

template <class State>
class RobotStateBackendAdapter final : public IRobotStateBackend {
public:
    explicit RobotStateBackendAdapter(Clock::time_point stamp) noexcept
        : state { stamp } { }

    auto initialize(Armor3D const& armor, Clock::time_point t) -> void override {
        state.initialize(armor, t);
    }

    auto predict(Clock::time_point t) -> void override { state.predict(t); }

    auto update(std::span<Armor3D const> armors) -> bool override { return state.update(armors); }

    [[nodiscard]] auto is_converged() const -> bool override { return state.is_converged(); }

    [[nodiscard]] auto get_snapshot() const -> Snapshot override { return state.get_snapshot(); }

    [[nodiscard]] auto distance() const -> double override { return state.distance(); }

private:
    State state;
};

[[nodiscard]] auto make_robot_state_backend(RobotStateBackendKind kind,
    IRobotStateBackend::Clock::time_point stamp) -> std::unique_ptr<IRobotStateBackend> {
    switch (kind) {
    case RobotStateBackendKind::Outpost:
        return std::make_unique<RobotStateBackendAdapter<OutpostRobotState>>(stamp);
    case RobotStateBackendKind::Regular:
        return std::make_unique<RobotStateBackendAdapter<RegularRobotState>>(stamp);
    }

    std::unreachable();
}

} // namespace rmcs::predictor::detail
