#include "robot_state.hpp"

using namespace rmcs::predictor;

struct RobotState::Impl {
    using EKF   = util::EKF<11, 4>;
    using Clock = std::chrono::steady_clock;
    using Stamp = Clock::time_point;

    explicit Impl(Armor3D const& armor, Stamp const& t)
        : device(armor.genre)
        , color(armor_color2camp_color(armor.color))
        , armor_num(EKFParameters::armor_num(armor.genre))
        , time_stamp(t)
        , initialized(true) {
        ekf = EKF { EKFParameters::x(armor), EKFParameters::P_initial_dig(device).asDiagonal() };
    }

    explicit Impl()
        : initialized(false) { }

    auto initialize(Armor3D const& armor, Stamp const& t) -> void {
        device    = armor.genre;
        color     = armor_color2camp_color(armor.color);
        armor_num = EKFParameters::armor_num(armor.genre);
        ekf = EKF { EKFParameters::x(armor), EKFParameters::P_initial_dig(device).asDiagonal() };
        time_stamp = t;

        initialized = true;
    }

    auto reset() -> void {
        initialized  = false;
        update_count = 0;
    }

    auto get_snapshot() const -> Snapshot { return { ekf, device, color, armor_num, time_stamp }; }

    auto distance() const -> double {
        auto x = ekf.x[0], y = ekf.x[2];
        return std::sqrt(x * x + y * y);
    }

    auto predict(Stamp const& t) -> void {
        if (!initialized) {
            time_stamp  = t;
            initialized = true;
            return;
        }

        auto dt = util::delta_time(t, time_stamp).count();
        ekf.predict(
            EKFParameters::f(dt), [dt](EKF::XVec const&) { return EKFParameters::F(dt); },
            EKFParameters::Q(device, dt));
        time_stamp = t;
    }

    auto update(Armor3D const& armor) -> void {
        if (!initialized) return;

        auto [id, error, valid] = match(armor);
        if (!valid) return;

        last_id = id;
        update_count++;

        auto const [pos_x, pos_y, pos_z] = armor.translation;
        auto const xyz                   = Eigen::Vector3d { pos_x, pos_y, pos_z };

        auto const& ypd = util::xyz2ypd(Eigen::Vector3d { pos_x, pos_y, pos_z });

        auto const [quat_x, quat_y, quat_z, quat_w] = armor.orientation;
        auto const& orientation = Eigen::Quaterniond { quat_w, quat_x, quat_y, quat_z };

        auto const& ypr = util::eulers(orientation);
        auto z          = EKF::ZVec {};
        z << ypd[0], ypd[1], ypd[2], ypr[0];

        ekf.update(
            z, [id, this](EKF::XVec const& x) { return EKFParameters::h(x, id, armor_num); },
            [id, this](EKF::XVec const& x) { return EKFParameters::H(x, id, armor_num); },
            EKFParameters::R(xyz, ypr, ypd), EKFParameters::x_add, EKFParameters::z_subtract);

        // 前哨站转速特判
        correct();
    }

    constexpr auto is_convergened() const -> bool {
        auto const r = ekf.x[8];
        auto const l = ekf.x[8] + ekf.x[9];

        auto const r_ok = (r > 0.05) && (r < 0.5);
        auto const l_ok = (l > 0.05) && (l < 0.5);

        if (r_ok && l_ok && update_count > 3) return true;

        if (r_ok && l_ok && device == DeviceId::OUTPOST && update_count > 10) return true;

        return false;
    }

    DeviceId device;
    CampColor color;
    int armor_num;

    EKF ekf;
    Stamp time_stamp;

    bool initialized;
    int last_id { 0 };
    int update_count { 0 };

    const double angle_error_threshold { 0.5 };
    // 前哨站转速特判
    constexpr auto correct() -> void {
        if (device == DeviceId::OUTPOST) {
            constexpr auto max_outpost_w = double { 2.51 };
            auto& w                      = ekf.x[7];
            if (std::abs(w) > 2.0) {
                w = w > 0 ? max_outpost_w : (-max_outpost_w);
            }
        }
    }

    constexpr auto calculate_armors(EKF::XVec const& x) const -> std::vector<Eigen::Vector4d> {
        auto armors = std::vector<Eigen::Vector4d> {};
        for (int i = 0; i < armor_num; i++) {
            auto angle = x[6];
            angle      = util::normalize_angle(angle + i * 2 * CV_PI / armor_num);

            auto xyz  = EKFParameters::h_armor_xyz(x, i, armor_num);
            auto xyza = Eigen::Vector4d { xyz[0], xyz[1], xyz[2], angle };
            armors.emplace_back(xyza);
        }
        return armors;
    }

    constexpr auto match(Armor3D const& armor) const -> MatchResult {
        if (!initialized) return { -1, 1e10, false };

        auto armors_xyza = calculate_armors(ekf.x);

        auto const& [pos_x, pos_y, pos_z] = armor.translation;
        const auto xyz                    = Eigen::Vector3d { pos_x, pos_y, pos_z };
        const auto orientation  = Eigen::Quaterniond { armor.orientation.w, armor.orientation.x,
            armor.orientation.y, armor.orientation.z };
        const auto ypr_in_world = util::eulers(orientation);
        const auto ypd_in_world = util::xyz2ypd(xyz);

        auto it =
            std::ranges::min_element(armors_xyza, [&](auto const& a_xyza, auto const& b_xyza) {
                auto get_error = [&](auto const& pred) {
                    auto ypd_pred = util::xyz2ypd(pred.template head<3>());
                    return std::abs(util::normalize_angle(ypr_in_world[0] - pred[3]))
                        + std::abs(util::normalize_angle(ypd_in_world[0] - ypd_pred[0]));
                };

                return get_error(a_xyza) < get_error(b_xyza);
            });

        int best_id = static_cast<int>(std::distance(armors_xyza.begin(), it));

        auto min_error = [&](const auto& pred) {
            auto ypd_pred = util::xyz2ypd(pred.template head<3>());
            return std::abs(util::normalize_angle(ypr_in_world[0] - pred[3]))
                + std::abs(util::normalize_angle(ypd_in_world[0] - ypd_pred[0]));
        }(*it);

        return { best_id, min_error, (min_error < angle_error_threshold) };
    }
};

RobotState::RobotState() noexcept
    : pimpl { std::make_unique<Impl>() } { }
RobotState::~RobotState() noexcept = default;

auto RobotState::initialize(
    rmcs::Armor3D const& armor, std::chrono::steady_clock::time_point const& t) -> void {
    return pimpl->initialize(armor, t);
}

auto RobotState::predict(std::chrono::steady_clock::time_point const& t) -> void {
    return pimpl->predict(t);
}

auto RobotState::match(Armor3D const& armor) const -> MatchResult { return pimpl->match(armor); }
auto RobotState::update(rmcs::Armor3D const& armor) -> void { return pimpl->update(armor); }

auto RobotState::is_convergened() const -> bool { return pimpl->is_convergened(); }

auto RobotState::get_snapshot() const -> Snapshot { return pimpl->get_snapshot(); }

auto RobotState::distance() const -> double { return pimpl->distance(); }
