#pragma once
#include "utility/robot/armor.hpp"

#include <opencv2/core/mat.hpp>

namespace rmcs {

inline const auto kRed      = cv::Scalar { 000, 000, 255, 255 };
inline const auto kGreen    = cv::Scalar { 000, 255, 000, 255 };
inline const auto kBlue     = cv::Scalar { 255, 000, 000, 255 };
inline const auto kYellow   = cv::Scalar { 000, 255, 255, 255 };
inline const auto kMagenta  = cv::Scalar { 255, 000, 255, 255 };
inline const auto kCyan     = cv::Scalar { 255, 255, 000, 255 };
inline const auto kBlack    = cv::Scalar { 000, 000, 000, 255 };
inline const auto kWhite    = cv::Scalar { 255, 255, 255, 255 };
inline const auto kGray     = cv::Scalar { 192, 192, 192, 255 };
inline const auto kDarkGray = cv::Scalar { 128, 128, 128, 255 };
inline const auto kBrown    = cv::Scalar { 000, 000, 128, 255 };
inline const auto kOrange   = cv::Scalar { 000, 165, 255, 255 };
inline const auto kPurple   = cv::Scalar { 128, 000, 128, 255 };
inline const auto kPink     = cv::Scalar { 203, 192, 255, 255 };

struct Canvas {
    struct Text {
        std::string content;
        cv::Point2i top_left;
        cv::Scalar color = kWhite;
    };
    struct Area {
        cv::Rect2i rect;
        cv::Scalar color = kWhite;
    };
    struct Point {
        cv::Point2i origin;
        int radius;

        cv::Scalar color = kWhite;
    };
    struct Line {
        cv::Point2i begin;
        cv::Point2i end;

        cv::Scalar color = kWhite;
    };
    struct ArmorShape {
        Armor2d shape;
        cv::Scalar color = kWhite;
    };

    cv::Mat& canvas;

    std::uint8_t transparency   = 255;
    std::uint8_t line_thickness = 1;

    auto draw(const ArmorShape&) -> void;
    auto draw(const Armor2d&) -> void;
    auto draw(const Lightbar2d&) -> void;
    auto draw(const cv::Rect2i&) -> void;
    auto draw(const Text&) -> void;
    auto draw(const Point&) -> void;
    auto draw(const Line&) -> void;
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
