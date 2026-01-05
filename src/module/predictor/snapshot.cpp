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

    Impl(EKF::XVec ekf_x, DeviceId device, CampColor color, int armor_num, TimePoint stamp) noexcept
        : ekf_x_ { std::move(ekf_x) }
        , device { device }
        , color { color }
        , armor_num { armor_num }
        , stamp { stamp } { }

    auto predict_at(TimePoint t) const -> EKF::XVec {
        double dt = util::delta_time(t, stamp).count();
        return EKFParameters::f(dt)(ekf_x_);
    }

    auto ekf_x() const -> EKF::XVec { return ekf_x_; }
    auto time_stamp() const -> TimePoint { return stamp; }

    auto predicted_armors(TimePoint t) const -> std::vector<Armor3D> {
        auto const& ekf_x = predict_at(t);
        auto _angle       = ekf_x[6];

        auto armors = std::vector<Armor3D> {};
        armors.reserve(armor_num);

        for (int id = 0; id < armor_num; ++id) {
            auto angle    = util::normalize_angle(_angle + id * 2 * std::numbers::pi / armor_num);
            auto position = EKFParameters::h_armor_xyz(ekf_x, id, armor_num);

            auto armor        = Armor3D {};
            armor.genre       = device;
            armor.color       = camp_color2armor_color(color);
            armor.id          = id;
            armor.translation = position;
            armor.orientation = util::euler_to_quaternion(angle, 15. / 180 * std::numbers::pi, 0);
            armors.emplace_back(armor);
        }
        return armors;
    }
};

Snapshot::Snapshot(
    EKF::XVec ekf_x, DeviceId device, CampColor color, int armor_num, TimePoint stamp) noexcept
    : pimpl { std::make_unique<Impl>(std::move(ekf_x), device, color, armor_num, stamp) } { }

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

<<<<<<< HEAD
auto Snapshot::predict_at(TimePoint t) const -> EKF::XVec { return pimpl->predict_at(t); }

=======
>>>>>>> fedd1fa (refactor(fire_control): optimize configuration initialization logic)
auto Snapshot::predicted_armors(TimePoint t) const -> std::vector<Armor3D> {
    return pimpl->predicted_armors(t);
}
