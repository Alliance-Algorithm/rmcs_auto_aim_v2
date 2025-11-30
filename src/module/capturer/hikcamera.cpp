#include "hikcamera.hpp"
#include "utility/image/image.details.hpp"

namespace rmcs::cap {

auto Hikcamera::wait_image() noexcept -> ImageResult {
    auto captured = read_image_with_timestamp();
    if (!captured.has_value()) {
        return std::unexpected { captured.error() };
    }

    auto image = std::make_unique<rmcs::Image>();
    image->details().set_mat(captured->mat);
    image->set_timestamp(captured->timestamp);

    return image;
}

}
