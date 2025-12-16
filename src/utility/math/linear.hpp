#pragma once
#include <concepts>

namespace rmcs {

template <class T>
concept translation_struct_trait = requires(T t) {
    { t.x } -> std::convertible_to<double>;
    { t.y } -> std::convertible_to<double>;
    { t.z } -> std::convertible_to<double>;
};
template <class T>
concept translation_object_trait = requires(T t) {
    { t.x() } -> std::convertible_to<double>;
    { t.y() } -> std::convertible_to<double>;
    { t.z() } -> std::convertible_to<double>;
};
template <class T>
concept translation_trait = translation_struct_trait<T> || translation_object_trait<T>;

template <class T>
concept orientation_struct_trait = requires(T t) {
    { t.x } -> std::convertible_to<double>;
    { t.y } -> std::convertible_to<double>;
    { t.z } -> std::convertible_to<double>;
    { t.w } -> std::convertible_to<double>;
};
template <class T>
concept orientation_object_trait = requires(T t) {
    { t.x() } -> std::convertible_to<double>;
    { t.y() } -> std::convertible_to<double>;
    { t.z() } -> std::convertible_to<double>;
    { t.w() } -> std::convertible_to<double>;
};
template <class T>
concept orientation_trait = orientation_struct_trait<T> || orientation_object_trait<T>;

namespace linear::details {
    template <typename Src, typename Dst>
    inline auto clone_translation(const Src& src, Dst& dst) noexcept {
        if constexpr (translation_object_trait<Src> && translation_object_trait<Dst>) {
            dst.x() = src.x();
            dst.y() = src.y();
            dst.z() = src.z();
        } else if constexpr (translation_struct_trait<Src> && translation_struct_trait<Dst>) {
            dst.x = src.x;
            dst.y = src.y;
            dst.z = src.z;
        } else if constexpr (translation_object_trait<Src> && translation_struct_trait<Dst>) {
            dst.x = src.x();
            dst.y = src.y();
            dst.z = src.z();
        } else if constexpr (translation_struct_trait<Src> && translation_object_trait<Dst>) {
            dst.x() = src.x;
            dst.y() = src.y;
            dst.z() = src.z;
        } else {
            static_assert(false, "clone_translation: unsupported trait combination");
        }
        return dst;
    }
    template <typename Src, typename Dst>
    inline auto clone_orientation(const Src& src, Dst& dst) noexcept {
        if constexpr (orientation_object_trait<Src> && orientation_object_trait<Dst>) {
            dst.x() = src.x();
            dst.y() = src.y();
            dst.z() = src.z();
            dst.w() = src.w();
        } else if constexpr (orientation_struct_trait<Src> && orientation_struct_trait<Dst>) {
            dst.x = src.x;
            dst.y = src.y;
            dst.z = src.z;
            dst.w = src.w;
        } else if constexpr (orientation_object_trait<Src> && orientation_struct_trait<Dst>) {
            dst.x = src.x();
            dst.y = src.y();
            dst.z = src.z();
            dst.w = src.w();
        } else if constexpr (orientation_struct_trait<Src> && orientation_object_trait<Dst>) {
            dst.x() = src.x;
            dst.y() = src.y;
            dst.z() = src.z;
            dst.w() = src.w;
        } else {
            static_assert(false, "clone_orientation: unsupported trait combination");
        }
        return dst;
    }
}

struct Scalar3d {
    double x = 0;
    double y = 0;
    double z = 0;

    /**
 * @brief 构造一个坐标为 (0.0, 0.0, 0.0) 的 Scalar3d 实例。
 *
 * 默认将成员 x、y、z 初始化为 0.0。
 */
constexpr Scalar3d() noexcept = default;
    /**
         * @brief 使用给定的坐标值初始化三维标量的各分量。
         *
         * @param x x 分量的初始值。
         * @param y y 分量的初始值。
         * @param z z 分量的初始值。
         */
        constexpr explicit Scalar3d(double x, double y, double z) noexcept
        : x { x }
        , y { y }
        , z { z } { }
    constexpr explicit Scalar3d(const translation_trait auto& t) noexcept {
        linear::details::clone_translation(t, *this);
    }
    auto operator=(const translation_trait auto& t) noexcept -> Scalar3d& {
        linear::details::clone_translation(t, *this);
        return *this;
    }
    auto copy_to(translation_trait auto& target) const noexcept -> void {
        linear::details::clone_translation(*this, target);
    }
    template <class T>
    auto make() const -> T {
        auto result = T {};
        return linear::details::clone_translation(*this, result);
    }
};
using Translation = Scalar3d;
using Vector3d    = Scalar3d;
using Direction3d = Scalar3d;

struct Orientation {
    double x = 0;
    double y = 0;
    double z = 0;
    double w = 1;

    /**
 * @brief 默认构造一个单位方向四元数。
 *
 * 构造后分量为 x = 0, y = 0, z = 0, w = 1。
 */
constexpr Orientation() noexcept = default;
    /**
         * @brief 使用给定分量初始化方向四元数。
         *
         * 使用参数 x、y、z、w 分别设置四元数的四个分量。
         *
         * @param x 四元数的 x 分量。
         * @param y 四元数的 y 分量。
         * @param z 四元数的 z 分量。
         * @param w 四元数的 w 分量。
         */
        constexpr explicit Orientation(double x, double y, double z, double w) noexcept
        : x { x }
        , y { y }
        , z { z }
        , w { w } { }
    constexpr explicit Orientation(const orientation_trait auto& q) noexcept {
        linear::details::clone_orientation(q, *this);
    }
    auto operator=(const orientation_trait auto& q) noexcept -> Orientation& {
        linear::details::clone_orientation(q, *this);
        return *this;
    }
    auto copy_to(orientation_trait auto& target) const noexcept -> void {
        linear::details::clone_orientation(*this, target);
    }
    template <class T>
    auto make() const -> T {
        auto result = T {};
        return linear::details::clone_orientation(*this, result);
    }
};

}