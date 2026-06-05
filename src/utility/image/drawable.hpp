#pragma once
#include "utility/image/image.hpp"
#include "utility/robot/armor.hpp"

namespace rmcs {

struct Text {
    std::string content;
    cv::Point2i center;
};
struct Area {
    cv::Rect2i rect;
    cv::Scalar color { 0, 0, 0, 255 };
};

struct Canvas {
    Image& canvas;

    std::uint8_t transparency   = 255;
    std::uint8_t line_thickness = 1;

    auto draw(const Armor2d&) -> void;
    auto draw(const Lightbar2d&) -> void;
    auto draw(const cv::Rect2i&) -> void;
    auto draw(const Text&) -> void;
};
template <class T>
concept drawable_trait = requires(Canvas& canvas) { canvas.draw(std::declval<T>()); };

struct IDrawable {
    virtual ~IDrawable() = default;
    virtual auto use(Canvas&) -> void { }
};
template <drawable_trait T>
struct Drawable : public IDrawable {
    T content;

    explicit Drawable(T content)
        : content { std::move(content) } { }

    ~Drawable() override = default;

    auto use(Canvas& canvas) -> void override { canvas.draw(content); }
};

}
