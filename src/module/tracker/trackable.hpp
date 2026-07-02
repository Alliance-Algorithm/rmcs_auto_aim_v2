#pragma once

#include "utility/clock.hpp"
#include "utility/math/linear.hpp"

namespace rmcs {

struct Trackable {
    using Unique = std::unique_ptr<Trackable>;
    using Points = std::vector<Point3d>;

    virtual ~Trackable() = default;

    virtual auto get_aimpoints() const -> Points  = 0;
    virtual auto get_direction() const -> Point3d = 0;

    virtual auto get_rotation_speed() const -> double = 0;

    virtual auto get_timestamp() const -> TimePoint = 0;
    virtual auto jump_into(double seconds) -> void  = 0;

    virtual auto clone() const -> Unique = 0;
};

template <class State>
struct Ins : public Trackable {
    explicit Ins(TimePoint timestamp, const State& state)
        : stamp { timestamp }
        , state { state } { }

    ~Ins() override = default;

    auto get_aimpoints() const -> std::vector<Point3d> override {
        constexpr auto kHasAimpoints = requires {
            { state.get_aimpoints() };
        };
        static_assert(kHasAimpoints, "State::get_aimpoints()");
        return state.get_aimpoints();
    }
    auto get_direction() const -> Point3d override {
        constexpr auto kHasDirection = requires {
            { state.get_direction() };
        };
        static_assert(kHasDirection, "State::get_direction()");
        return state.get_direction();
    }

    auto get_rotation_speed() const -> double override {
        constexpr auto kHasRotationSpeed = requires {
            { state.get_rotation_speed() };
        };
        static_assert(kHasRotationSpeed, "State::get_rotation_speed()");
        return state.get_rotation_speed();
    }

    auto get_timestamp() const -> TimePoint override { return stamp; }
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

constexpr auto make_trackable = []<class State>(Timestamp stamp, const State& state) {
    return std::make_unique<Ins<State>>(stamp, state);
};

}
