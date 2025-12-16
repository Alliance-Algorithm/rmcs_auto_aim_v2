#include "armor.hpp"
#include "utility/image/image.details.hpp"
#include <opencv2/imgproc.hpp>

namespace rmcs::util {

auto draw(Image& canvas, const Armor2D& armor) noexcept -> void {
    auto& opencv_mat = canvas.details().mat;

    auto color = cv::Scalar { 0, 0, 0 };
    /*  */ if (armor.color == ArmorColor::BLUE) {
        color = { 255, 0, 0 };
    } else if (armor.color == ArmorColor::RED) {
        color = { 0, 0, 255 };
    } else if (armor.color == ArmorColor::MIX) {
        color = { 255, 0, 255 };
    }

    auto genre = get_enum_name(armor.genre);
    auto shape = get_enum_name(armor.shape);

    auto first    = cv::Point2f {};
    auto prev     = cv::Point2f {};
    auto is_first = true;

    for (const auto& point : armor.corners()) {
        cv::circle(opencv_mat, point, 2, color, -1);

        if (!is_first) {
            cv::line(opencv_mat, prev, point, color, 2, cv::LINE_AA);
        } else {
            first    = point;
            is_first = false;
        }
        prev = point;
    }

    if (!is_first) {
        cv::line(opencv_mat, prev, first, color, 2, cv::LINE_AA);
    }

    const auto thickness = 1;
    const auto font      = cv::FONT_HERSHEY_SIMPLEX;
    const auto scale     = 0.6;
    const auto white     = cv::Scalar { 255, 255, 255 };

    cv::putText(opencv_mat, "TR", armor.tr, font, scale, white, thickness, cv::LINE_AA);
    cv::putText(opencv_mat, "BL", armor.bl, font, scale, white, thickness, cv::LINE_AA);
    cv::putText(opencv_mat, "BR", armor.br, font, scale, white, thickness, cv::LINE_AA);

    auto info = std::format("{:.2f} {} {}", armor.confidence, genre, shape);
    cv::putText(opencv_mat, info, armor.tl + cv::Point2f { 0, -5 }, font, scale, white, thickness,
        cv::LINE_AA);
}

}
