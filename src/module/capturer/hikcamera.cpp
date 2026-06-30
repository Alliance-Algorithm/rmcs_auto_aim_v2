#include "hikcamera.hpp"

namespace rmcs::cap {

auto Hikcamera::wait_image() noexcept -> ImageResult {
    auto captured = read_image_with_timestamp();
    if (!captured.has_value()) {
        return std::unexpected { captured.error() };
    }

    auto image       = std::make_unique<rmcs::Image>();
    image->mat       = std::move(captured->mat);
    image->timestamp = captured->timestamp;

    return image;
}

}
