#include "snapshot.hpp"

#include <type_traits>
#include <variant>

#include "module/predictor/ekf_parameter.hpp"
#include "module/predictor/outpost_ekf_parameter.hpp"
#include "utility/math/conversion.hpp"
#include "utility/robot/armor.hpp"
#include "utility/time.hpp"

using namespace rmcs::predictor;
using TimePoint = std::chrono::steady_clock::time_point;

namespace {
struct NormalSnapshotState {
    Snapshot::NormalEKF::XVec x;
};

struct OutpostSnapshotState {
    Snapshot::OutpostEKF::XVec x;
    int spin_sign;
    int order_idx;
};
}

struct Snapshot::Impl {
    using InternalState = std::variant<NormalSnapshotState, OutpostSnapshotState>;

    InternalState state_;
    DeviceId device;
    CampColor color;
    int armor_num;
    TimePoint stamp;

    Impl(InternalState state, DeviceId device, CampColor color, int armor_num,
        TimePoint stamp) noexcept
        : state_ { std::move(state) }
        , device { device }
        , color { color }
        , armor_num { armor_num }
        , stamp { stamp } { }

    static auto normalized_spin_sign(int spin_sign) -> int { return spin_sign >= 0 ? +1 : -1; }

    static auto to_public_state(InternalState const& state) -> Snapshot::State {
        return std::visit(
            [](auto const& typed_state) -> Snapshot::State { return typed_state.x; }, state);
    }

    static auto kinematics_of(InternalState const& state) -> Snapshot::Kinematics {
        return std::visit(
            [](auto const& typed_state) -> Snapshot::Kinematics {
                using TypedState = std::decay_t<decltype(typed_state)>;
                if constexpr (std::is_same_v<TypedState, NormalSnapshotState>) {
                    return { Eigen::Vector3d {
                                 typed_state.x[0], typed_state.x[2], typed_state.x[4] },
                        typed_state.x[7] };
                } else {
                    auto angular_velocity =
                        typed_state.spin_sign >= 0 ? kOutpostAngularSpeed : -kOutpostAngularSpeed;
                    return { Eigen::Vector3d {
                                 typed_state.x[0], typed_state.x[2], typed_state.x[4] },
                        angular_velocity };
                }
            },
            state);
    }

    auto predict_state_at(TimePoint t) const -> InternalState {
        double dt = util::delta_time(t, stamp).count();
        return std::visit(
            [dt](auto const& typed_state) -> InternalState {
                using TypedState = std::decay_t<decltype(typed_state)>;
                if constexpr (std::is_same_v<TypedState, NormalSnapshotState>) {
                    return NormalSnapshotState { EKFParameters::f(dt)(typed_state.x) };
                } else {
                    return OutpostSnapshotState {
                        OutpostEKFParameters::f(dt, typed_state.spin_sign)(typed_state.x),
                        typed_state.spin_sign,
                        typed_state.order_idx,
                    };
                }
            },
            state_);
    }

    auto state() const -> Snapshot::State { return to_public_state(state_); }
    auto time_stamp() const -> TimePoint { return stamp; }
    auto kinematics() const -> Snapshot::Kinematics { return kinematics_of(state_); }
    auto kinematics_at(TimePoint t) const -> Snapshot::Kinematics {
        return kinematics_of(predict_state_at(t));
    }

    auto predicted_armors(TimePoint t) const -> std::vector<Armor3D> {
        auto predicted_state = predict_state_at(t);

        auto armors = std::vector<Armor3D> {};
        armors.reserve(armor_num);

        std::visit(
            [&](auto const& typed_state) {
                using TypedState = std::decay_t<decltype(typed_state)>;
                for (int id = 0; id < armor_num; ++id) {
                    auto armor  = Armor3D {};
                    armor.genre = device;
                    armor.color = camp_color2armor_color(color);
                    armor.id    = id;

                    if constexpr (std::is_same_v<TypedState, NormalSnapshotState>) {
                        auto angle = EKFParameters::armor_yaw(device, typed_state.x, id);
                        auto position =
                            EKFParameters::h_armor_xyz(device, typed_state.x, id, armor_num);
                        armor.translation = position;
                        armor.orientation =
                            util::euler_to_quaternion(angle, predicted_armor_pitch(device), 0);
                    } else {
                        auto angle    = OutpostEKFParameters::armor_yaw(typed_state.x, id);
                        auto position = OutpostEKFParameters::h_armor_xyz(
                            typed_state.x, id, typed_state.order_idx);
                        armor.translation = position;
                        armor.orientation =
                            util::euler_to_quaternion(angle, predicted_armor_pitch(device), 0);
                    }

                    armors.emplace_back(armor);
                }
            },
            predicted_state);
        return armors;
    }

private:
    static auto predicted_armor_pitch(DeviceId device) -> double {
        switch (device) {
        case DeviceId::OUTPOST:
            return kPredictedOutpostArmorPitch;
        default:
            return kPredictedOtherArmorPitch;
        }
    }
};

Snapshot::Snapshot(NormalEKF::XVec ekf_x, DeviceId device, CampColor color, int armor_num,
    TimePoint stamp) noexcept
    : pimpl { std::make_unique<Impl>(
          NormalSnapshotState { std::move(ekf_x) }, device, color, armor_num, stamp) } { }

Snapshot::Snapshot(OutpostEKF::XVec ekf_x, CampColor color, int armor_num, TimePoint stamp,
    int outpost_spin_sign, int outpost_order_idx) noexcept
    : pimpl { std::make_unique<Impl>(
          OutpostSnapshotState {
              std::move(ekf_x), Impl::normalized_spin_sign(outpost_spin_sign), outpost_order_idx },
          DeviceId::OUTPOST, color, armor_num, stamp) } { }

Snapshot::Snapshot(Snapshot const& other)
    : pimpl { std::make_unique<Impl>(*other.pimpl) } { }

Snapshot::Snapshot(Snapshot&&) noexcept = default;
Snapshot& Snapshot::operator=(Snapshot const& other) {
    if (this != &other) {
        pimpl = std::make_unique<Impl>(*other.pimpl);
    }
    return *this;
}
Snapshot& Snapshot::operator=(Snapshot&&) noexcept = default;
Snapshot::~Snapshot() noexcept                     = default;

auto Snapshot::state() const -> State { return pimpl->state(); }

auto Snapshot::time_stamp() const -> TimePoint { return pimpl->time_stamp(); }

auto Snapshot::predict_state_at(TimePoint t) const -> State {
    return pimpl->to_public_state(pimpl->predict_state_at(t));
}

auto Snapshot::kinematics() const -> Kinematics { return pimpl->kinematics(); }

auto Snapshot::kinematics_at(TimePoint t) const -> Kinematics { return pimpl->kinematics_at(t); }

auto Snapshot::predicted_armors(TimePoint t) const -> std::vector<Armor3D> {
    return pimpl->predicted_armors(t);
}
