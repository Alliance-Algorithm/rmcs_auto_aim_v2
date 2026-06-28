#pragma once

#include "utility/clock.hpp"
#include "utility/math/linear.hpp"

namespace rmcs {

struct Trackable {
    using Unique = std::unique_ptr<Trackable>;
    using Points = std::vector<Point3d>;

    virtual ~Trackable() = default;

    virtual auto aimpoints() const -> Points  = 0;
    virtual auto direction() const -> Point3d = 0;

    virtual auto timestamp() const -> TimePoint    = 0;
    virtual auto jump_into(double seconds) -> void = 0;

    virtual auto clone() const -> Unique = 0;
};

template <class State>
struct Ins : public Trackable {
    explicit Ins(TimePoint timestamp, const State& state)
        : stamp { timestamp }
        , state { state } { }

    ~Ins() override = default;

    auto aimpoints() const -> Points override {
        constexpr auto kHasAimpoints = requires {
            { state.aimpoints() };
        };
        static_assert(kHasAimpoints, "State::aimpoint()");
        return state.aimpoints();
    }
    auto direction() const -> Point3d override {
        constexpr auto kHasDirection = requires {
            { state.direction() };
        };
        static_assert(kHasDirection, "State::direction()");
        return state.direction();
    }

    auto timestamp() const -> TimePoint override { return stamp; }
    auto jump_into(double seconds) -> void override {
        constexpr auto kHasTransition = requires {
            { state.transition(0.) };
        };
        static_assert(kHasTransition, "State::transition(seconds)");
        state.transition(seconds);
        stamp += std::chrono::duration_cast<TimePoint::duration>(
            std::chrono::duration<double> { seconds });
    }

    auto clone() const -> Unique override { return std::make_unique<Ins>(stamp, state); }

private:
    TimePoint stamp;
    State state;
};

}
