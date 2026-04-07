#include "snapshot.hpp"

#include "module/predictor/ekf_parameter.hpp"
#include "utility/math/conversion.hpp"
#include "utility/robot/armor.hpp"
#include "utility/time.hpp"

using namespace rmcs::predictor;
using TimePoint = std::chrono::steady_clock::time_point;

struct Snapshot::Impl {
    EKF::XVec ekf_x_;
    DeviceId device;
    CampColor color;
    int armor_num;
    TimePoint stamp;
    int outpost_order_idx;

    Impl(EKF::XVec ekf_x, DeviceId device, CampColor color, int armor_num, TimePoint stamp,
        int outpost_order_idx) noexcept
        : ekf_x_ { std::move(ekf_x) }
        , device { device }
        , color { color }
        , armor_num { armor_num }
        , stamp { stamp }
        , outpost_order_idx { outpost_order_idx } { }

    auto predict_at(TimePoint t) const -> EKF::XVec {
        double dt = util::delta_time(t, stamp).count();
        return EKFParameters::f(device, dt)(ekf_x_);
    }

    auto ekf_x() const -> EKF::XVec { return ekf_x_; }
    auto time_stamp() const -> TimePoint { return stamp; }

    auto predicted_armors(TimePoint t) const -> std::vector<Armor3D> {
        auto const& ekf_x = predict_at(t);

        auto armors = std::vector<Armor3D> {};
        armors.reserve(armor_num);

        for (int id = 0; id < armor_num; ++id) {
            auto angle = EKFParameters::armor_yaw(device, ekf_x, id);
            auto position =
                EKFParameters::h_armor_xyz(device, ekf_x, id, armor_num, outpost_order_idx);

            auto armor        = Armor3D {};
            armor.genre       = device;
            armor.color       = camp_color2armor_color(color);
            armor.id          = id;
            armor.translation = position;
            armor.orientation = util::euler_to_quaternion(angle, predicted_armor_pitch(device), 0);
            armors.emplace_back(armor);
        }
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

Snapshot::Snapshot(EKF::XVec ekf_x, DeviceId device, CampColor color, int armor_num,
    TimePoint stamp, int outpost_order_idx) noexcept
    : pimpl { std::make_unique<Impl>(
          std::move(ekf_x), device, color, armor_num, stamp, outpost_order_idx) } { }

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

auto Snapshot::ekf_x() const -> EKF::XVec { return pimpl->ekf_x(); }

auto Snapshot::time_stamp() const -> TimePoint { return pimpl->time_stamp(); }

auto Snapshot::predict_at(TimePoint t) const -> EKF::XVec { return pimpl->predict_at(t); }

auto Snapshot::predicted_armors(TimePoint t) const -> std::vector<Armor3D> {
    return pimpl->predicted_armors(t);
}
