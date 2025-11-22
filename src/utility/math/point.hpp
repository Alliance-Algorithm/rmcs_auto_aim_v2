#pragma once
#include <concepts>

namespace rmcs {

template <class T>
concept point2d_struct_trait = requires(T t) {
    { t.x } -> std::convertible_to<double>;
    { t.y } -> std::convertible_to<double>;
};
template <class T>
concept point2d_object_trait = requires(T t) {
    { t.x() } -> std::convertible_to<double>;
    { t.y() } -> std::convertible_to<double>;
};
template <class T>
concept point2d_trait = point2d_struct_trait<T> || point2d_object_trait<T>;

template <class T>
concept point3d_struct_trait = requires(T t) {
    { t.x } -> std::convertible_to<double>;
    { t.y } -> std::convertible_to<double>;
    { t.z } -> std::convertible_to<double>;
};
template <class T>
concept point3d_object_trait = requires(T t) {
    { t.x() } -> std::convertible_to<double>;
    { t.y() } -> std::convertible_to<double>;
    { t.z() } -> std::convertible_to<double>;
};
template <class T>
concept point3d_trait = point3d_struct_trait<T> || point3d_object_trait<T>;

namespace point::details {
    template <typename Src, typename Dst>
    inline auto clone_point2d(const Src& src, Dst& dst) noexcept -> Dst {
        if constexpr (point2d_object_trait<Src> && point2d_object_trait<Dst>) {
            dst.x() = src.x();
            dst.y() = src.y();
        } else if constexpr (point2d_struct_trait<Src> && point2d_struct_trait<Dst>) {
            dst.x = src.x;
            dst.y = src.y;
        } else if constexpr (point2d_object_trait<Src> && point2d_struct_trait<Dst>) {
            dst.x = src.x();
            dst.y = src.y();
        } else if constexpr (point2d_struct_trait<Src> && point2d_object_trait<Dst>) {
            dst.x() = src.x;
            dst.y() = src.y;
        } else {
            static_assert(false, "clone_point2d: unsupported trait combination");
        }
        return dst;
    }

    template <typename Src, typename Dst>
    inline auto clone_point3d(const Src& src, Dst& dst) noexcept -> Dst {
        if constexpr (point3d_object_trait<Src> && point3d_object_trait<Dst>) {
            dst.x() = src.x();
            dst.y() = src.y();
            dst.z() = src.z();
        } else if constexpr (point3d_struct_trait<Src> && point3d_struct_trait<Dst>) {
            dst.x = src.x;
            dst.y = src.y;
            dst.z = src.z;
        } else if constexpr (point3d_object_trait<Src> && point3d_struct_trait<Dst>) {
            dst.x = src.x();
            dst.y = src.y();
            dst.z = src.z();
        } else if constexpr (point3d_struct_trait<Src> && point3d_object_trait<Dst>) {
            dst.x() = src.x;
            dst.y() = src.y;
            dst.z() = src.z;
        } else {
            static_assert(false, "clone_point3d: unsupported trait combination");
        }
        return dst;
    }
}

struct Point2D {
    double x = 0;
    double y = 0;

    constexpr explicit Point2D() noexcept = default;
    constexpr explicit Point2D(const point2d_trait auto& p) noexcept {
        point::details::clone_point2d(p, *this);
    }
    auto operator=(const point2d_trait auto& p) noexcept -> Point2D& {
        point::details::clone_point2d(p, *this);
        return *this;
    }
    auto copy_to(point2d_trait auto& target) const noexcept -> void {
        point::details::clone_point2d(*this, target);
    }
    template <class T>
    auto make() const -> T {
        auto result = T {};
        return point::details::clone_point2d(*this, result);
    }
};

struct Point3D {
    double x = 0;
    double y = 0;
    double z = 0;

    constexpr explicit Point3D() noexcept = default;
    constexpr explicit Point3D(const point3d_trait auto& p) noexcept {
        point::details::clone_point3d(p, *this);
    }
    auto operator=(const point3d_trait auto& p) noexcept -> Point3D& {
        point::details::clone_point3d(p, *this);
        return *this;
    }
    auto copy_to(point3d_trait auto& target) const noexcept -> void {
        point::details::clone_point3d(*this, target);
    }
    template <class T>
    auto make() const -> T {
        auto result = T {};
        return point::details::clone_point3d(*this, result);
    }
};

}
