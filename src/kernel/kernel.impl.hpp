#include "concept.hpp"
#include "kernel.hpp"
#include "modules/debug/recorder.hpp"
#include "modules/debug/visualization.hpp"

#include <opencv2/core/mat.hpp>

#include <expected>

namespace rmcs {

template <class Capturer, class Identifier, class Tracker>
class Kernel {
public:
    static_assert(
        concept_capturer<Capturer>,
        "[KERNEL CONCEPT]: Template parameter 'Capturer' does not satisfy the concept_capturer "
        "requirement: "
        "It must have a member function 'cv::Mat read(std::chrono::seconds)' callable as "
        "'capturer.read(std::chrono::seconds(5))'.");

    static_assert(concept_identifier<Identifier>, "[KERNEL CONCEPT]:");

    explicit Kernel(std::unique_ptr<Capturer> capturer) noexcept
        : capturer_{std::move(capturer)} {};

    auto run() -> void {
        while (true) {}
    }

    auto set_config(const std::string& path) noexcept -> void { config_path_ = path; }

    auto read_config() noexcept -> std::expected<void, std::string> {
        return std::unexpected<std::string>{""};
    }

private:
    std::unique_ptr<Capturer> capturer_;

    std::string config_path_;

    debug::Visualization visualzation_;
    debug::Recorder recorder_;
};

} // namespace rmcs
