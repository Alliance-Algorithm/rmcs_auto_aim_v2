#pragma once

#include <deque>
#include <limits>

namespace rmcs {

class RuneEnergyFitter {
public:
    struct FitResult {
        double C     = 0.0;
        double v     = 0.0;
        double a     = 0.0;
        double omega = 0.0;
        double phi   = 0.0;
        double cost  = std::numeric_limits<double>::max();
        bool valid   = false;
    };

    struct LinearResult {
        double C     = 0.0;
        double speed = 0.0;
        double cost  = std::numeric_limits<double>::max();
        bool valid   = false;
    };

    void push(double timestamp, double theta);
    void reset();

    auto fit_linear() -> const LinearResult&;
    auto fit_sine() -> const FitResult&;

    auto base_t() const { return base_t_; }

    static constexpr double kWindowSeconds = 6.0;
    static constexpr double kMinFitSeconds = 1.5;

    ~RuneEnergyFitter();

private:
    struct Point {
        double t;
        double theta;
    };
    std::deque<Point> buffer_;

    FitResult    sine_result_;
    LinearResult linear_result_;

    double base_t_ = 0.0;

    template <typename Pred>
    static auto compute_raw_cost(
        const std::deque<Point>& buffer, Pred&& pred_fn) -> double;
};

} // namespace rmcs
