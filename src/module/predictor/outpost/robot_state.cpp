#include "robot_state.hpp"

#include "module/predictor/model/outpost.hpp"
#include "module/predictor/outpost/snapshot.hpp"
#include "utility/math/angle.hpp"
#include "utility/robot/constant.hpp"

#include <cmath>
#include <limits>
#include <memory>
#include <vector>

namespace rmcs::predictor {

struct OutpostRobotState::Impl {
    std::unique_ptr<OutpostModel> model;

    auto predict(double dt) const -> void {
        if (!(dt > 0.0) || !model) return;
        model->predict(dt);
    }

    auto update(std::span<Armor3d const> armors) -> bool {
        if (armors.empty()) return false;

        if (!model) {
            model = std::make_unique<OutpostModel>(armors.front());
        } else {
            model->correct(armors.front());
        }

        return true;
    }

    auto is_converged() const noexcept -> bool {
        if (!model) return false;

        return true;
    }

    auto get_snapshot(TimePoint stamp) const -> std::optional<Snapshot> {
        if (!model) return std::nullopt;

        const auto full = model->full();
        auto armors     = std::vector<Armor3d> { full.begin(), full.end() };
        return Snapshot { OutpostSnapshot { model->state(), std::move(armors), stamp } };
    }

    auto distance() const -> double {
        if (!model) return std::numeric_limits<double>::infinity();
        const auto state = model->state();
        return std::hypot(state.x, state.y);
    }
};

OutpostRobotState::OutpostRobotState() noexcept
    : pimpl { std::make_unique<Impl>() } { }

OutpostRobotState::~OutpostRobotState() noexcept = default;

auto OutpostRobotState::predict(double dt) -> void { return pimpl->predict(dt); }

auto OutpostRobotState::update(std::span<Armor3d const> armors) -> bool {
    return pimpl->update(armors);
}

auto OutpostRobotState::is_converged() const -> bool { return pimpl->is_converged(); }

auto OutpostRobotState::get_snapshot(TimePoint stamp) const -> std::optional<Snapshot> {
    return pimpl->get_snapshot(stamp);
}

auto OutpostRobotState::distance() const -> double { return pimpl->distance(); }

} // namespace rmcs::predictor
