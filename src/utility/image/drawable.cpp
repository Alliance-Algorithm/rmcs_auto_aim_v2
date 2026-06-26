#include "drawable.hpp"
#include "utility/image/image.details.hpp"

#include <format>
#include <opencv2/imgproc.hpp>

namespace rmcs {

namespace details {

    auto to_scalar(ArmorColor color, std::uint8_t alpha) -> cv::Scalar {
        const auto a = static_cast<double>(alpha);
        auto bgra    = cv::Scalar { 0, 0, 0, a };
        if (color == ArmorColor::BLUE) {
            bgra = { 255, 0, 0, a };
        } else if (color == ArmorColor::RED) {
            bgra = { 0, 0, 255, a };
        } else if (color == ArmorColor::MIX) {
            bgra = { 255, 0, 255, a };
        }
        return bgra;
    }

}

auto Canvas::draw(const ArmorShape& armor) -> void {
    auto& mat = canvas.details().mat;

    const auto& color = armor.color;
    const auto& shape = armor.shape;

    cv::circle(mat, shape.tl, 2, color, -1);
    cv::circle(mat, shape.tr, 2, color, -1);
    cv::circle(mat, shape.br, 2, color, -1);
    cv::circle(mat, shape.bl, 2, color, -1);

    cv::line(mat, shape.tl, shape.tr, color, line_thickness, cv::LINE_AA);
    cv::line(mat, shape.tr, shape.br, color, line_thickness, cv::LINE_AA);
    cv::line(mat, shape.br, shape.bl, color, line_thickness, cv::LINE_AA);
    cv::line(mat, shape.bl, shape.tl, color, line_thickness, cv::LINE_AA);
}

auto Canvas::draw(const Armor2d& armor) -> void {
    auto& mat = canvas.details().mat;

    const auto color = details::to_scalar(armor.color, transparency);
    const auto white = cv::Scalar { 255, 255, 255, static_cast<double>(transparency) };
    const auto font  = cv::FONT_HERSHEY_SIMPLEX;
    const auto line  = cv::LINE_AA;

    const auto font_size = 0.4;

    cv::circle(mat, armor.tl, 2, color, -1);
    cv::circle(mat, armor.tr, 2, color, -1);
    cv::circle(mat, armor.br, 2, color, -1);
    cv::circle(mat, armor.bl, 2, color, -1);

    cv::line(mat, armor.tl, armor.tr, color, line_thickness, line);
    cv::line(mat, armor.tr, armor.br, color, line_thickness, line);
    cv::line(mat, armor.br, armor.bl, color, line_thickness, line);
    cv::line(mat, armor.bl, armor.tl, color, line_thickness, line);

    const auto text_thickness = 1;
    const auto arrow_length   = 5.0f;

    const auto tip_tl = armor.tl + cv::Point2f { -arrow_length, -arrow_length };
    const auto tip_tr = armor.tr + cv::Point2f { +arrow_length, -arrow_length };
    const auto tip_bl = armor.bl + cv::Point2f { -arrow_length, +arrow_length };
    const auto tip_br = armor.br + cv::Point2f { +arrow_length, +arrow_length };

    cv::arrowedLine(mat, tip_tl, armor.tl, white, text_thickness, cv::LINE_AA);
    cv::arrowedLine(mat, tip_tr, armor.tr, white, text_thickness, cv::LINE_AA);
    cv::arrowedLine(mat, tip_bl, armor.bl, white, text_thickness, cv::LINE_AA);
    cv::arrowedLine(mat, tip_br, armor.br, white, text_thickness, cv::LINE_AA);

    const auto genre = get_enum_name(armor.genre);
    const auto shape = get_enum_name(armor.shape);
    const auto info  = std::format("{:.2f} {} {}", armor.confidence, genre, shape);
    cv::putText(mat, info, armor.tl + cv::Point2f { 0, -15 }, font, font_size, white,
        text_thickness, cv::LINE_AA);
}

auto Canvas::draw(const Lightbar2d& lightbar) -> void {
    auto& mat  = canvas.details().mat;
    auto color = lightbar.draw_color.value_or(details::to_scalar(lightbar.color, transparency));

    cv::circle(mat, lightbar.upper.make<cv::Point2f>(), 2, color, -1, cv::LINE_AA);
    cv::circle(mat, lightbar.lower.make<cv::Point2f>(), 2, color, -1, cv::LINE_AA);
}

auto Canvas::draw(const cv::Rect2i& rect) -> void {
    auto& mat  = canvas.details().mat;
    auto color = cv::Scalar { 127, 127, 127, static_cast<double>(transparency) };

    cv::rectangle(mat, rect, color, line_thickness, cv::LINE_AA);
}

auto Canvas::draw(const Text& text) -> void {
    auto& mat = canvas.details().mat;

    const auto font  = cv::FONT_HERSHEY_SIMPLEX;
    const auto scale = 0.5;

    const auto font_color = text.color;
    const auto back_color = cv::Scalar { 0, 0, 0 };

    auto baseline   = 0;
    const auto size = cv::getTextSize(text.content, font, scale, line_thickness, &baseline);
    const auto org  = cv::Point2i {
        text.top_left.x,
        text.top_left.y + size.height,
    };

    cv::putText(mat, text.content, org + cv::Point2i { 1, 1 }, font, scale, back_color,
        line_thickness + 2, cv::LINE_AA);
    cv::putText(mat, text.content, org, font, scale, font_color, line_thickness, cv::LINE_AA);
}

auto Canvas::draw(const Point& point) -> void {
    auto& mat = canvas.details().mat;
    cv::circle(mat, point.origin, point.radius, point.color, -1, cv::LINE_AA);
}

}
