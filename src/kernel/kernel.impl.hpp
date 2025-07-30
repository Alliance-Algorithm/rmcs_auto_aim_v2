#include "kernel.hpp"

#include <opencv2/core/mat.hpp>
#include <rclcpp/node.hpp>

namespace rmcs::internal {

template <class T>
concept concept_capturer = requires(T& capturer) {
    { capturer.read(std::chrono::seconds(5)) } -> std::same_as<cv::Mat>;
};

template <class Capturer, class Identifier, class Tracker>
class Kernel {
public:
    static_assert(
        concept_capturer<Capturer>,
        "[KERNEL CONCEPT]: Template parameter 'Capturer' does not satisfy the concept_capturer "
        "requirement: "
        "It must have a member function 'cv::Mat read(std::chrono::seconds)' callable as "
        "'capturer.read(std::chrono::seconds(5))'.");

    explicit Kernel(std::unique_ptr<Capturer> capturer) noexcept
        : capturer_{std::move(capturer)} {};

private:
    std::unique_ptr<Capturer> capturer_;
};

} // namespace rmcs::internal
