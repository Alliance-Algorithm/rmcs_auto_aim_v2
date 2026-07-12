#include "rune_energy_fitter.hpp"

#include <eigen3/Eigen/Dense>

#include <cmath>

namespace rmcs {

RuneEnergyFitter::~RuneEnergyFitter() = default;

void RuneEnergyFitter::push(double timestamp, double theta) {
    if (buffer_.empty()) {
        base_t_ = timestamp;
    }
    buffer_.push_back({timestamp - base_t_, theta});

    while (!buffer_.empty() && buffer_.back().t - buffer_.front().t > kWindowSeconds)
        buffer_.pop_front();

    sine_result_.valid   = false;
    linear_result_.valid = false;
}

void RuneEnergyFitter::reset() {
    buffer_.clear();
    sine_result_   = FitResult{};
    linear_result_ = LinearResult{};
    base_t_        = 0.0;
}

template <typename Pred>
auto RuneEnergyFitter::compute_raw_cost(
    const std::deque<Point>& buffer, Pred&& pred_fn) -> double {

    double sum = 0.0;
    for (const auto& pt : buffer) {
        double diff = pt.theta - pred_fn(pt.t);
        sum += diff * diff;
    }
    return sum / static_cast<double>(buffer.size());
}

auto RuneEnergyFitter::fit_linear() -> const LinearResult& {
    if (linear_result_.valid || buffer_.size() < 2)
        return linear_result_;
    if (buffer_.back().t - buffer_.front().t < kMinFitSeconds)
        return linear_result_;

    const auto N = static_cast<double>(buffer_.size());

    double sum_t = 0.0, sum_theta = 0.0, sum_tt = 0.0, sum_t_theta = 0.0;
    for (const auto& pt : buffer_) {
        double dt = pt.t;
        double th = pt.theta;
        sum_t += dt;
        sum_theta += th;
        sum_tt += dt * dt;
        sum_t_theta += dt * th;
    }

    double denom = N * sum_tt - sum_t * sum_t;
    if (std::abs(denom) < 1e-12) return linear_result_;

    double speed = (N * sum_t_theta - sum_t * sum_theta) / denom;
    double C     = (sum_theta - speed * sum_t) / N;

    auto pred = [&](double dt) { return C + speed * dt; };
    double cost = compute_raw_cost(buffer_, pred);

    linear_result_ = {C, speed, cost, true};
    return linear_result_;
}

auto RuneEnergyFitter::fit_sine() -> const FitResult& {
    if (sine_result_.valid || buffer_.size() < 2)
        return sine_result_;
    if (buffer_.back().t - buffer_.front().t < kMinFitSeconds)
        return sine_result_;

    const auto N = static_cast<Eigen::Index>(buffer_.size());
    Eigen::MatrixXd X(N, 4);
    Eigen::VectorXd y(N);
    for (Eigen::Index i = 0; i < N; ++i) {
        double t = buffer_[i].t;
        y(i)     = buffer_[i].theta;
        X(i, 0)  = 1.0;
        X(i, 1)  = t;
    }

    static constexpr int    kSweepSteps = 41;
    static constexpr double kOmegaMin   = 1.80;
    static constexpr double kOmegaMax   = 2.20;

    double best_mse   = std::numeric_limits<double>::max();
    double best_C = 0, best_v = 0, best_A = 0, best_B = 0, best_omega = 0;

    for (int s = 0; s <= kSweepSteps; ++s) {
        double omega = kOmegaMin + s * (kOmegaMax - kOmegaMin) / kSweepSteps;

        for (Eigen::Index i = 0; i < N; ++i) {
            double t = buffer_[i].t;
            X(i, 2) = std::cos(omega * t);
            X(i, 3) = std::sin(omega * t);
        }

        Eigen::Vector4d coeff = X.colPivHouseholderQr().solve(y);
        double mse = (y - X * coeff).squaredNorm() / N;
        if (mse < best_mse) {
            best_mse   = mse;
            best_C     = coeff(0);
            best_v     = coeff(1);
            best_A     = coeff(2);
            best_B     = coeff(3);
            best_omega = omega;
        }
    }

    double a   = best_omega * std::sqrt(best_A * best_A + best_B * best_B);
    double phi = std::atan2(best_B, -best_A);

    auto pred = [&](double t) {
        return best_C + best_v * t - a / best_omega * std::cos(best_omega * t + phi);
    };
    double cost = compute_raw_cost(buffer_, pred);

    sine_result_ = {best_C, best_v, a, best_omega, phi, cost, true};
    if (a < 0.6) sine_result_.valid = false;
    return sine_result_;
}

} // namespace rmcs
