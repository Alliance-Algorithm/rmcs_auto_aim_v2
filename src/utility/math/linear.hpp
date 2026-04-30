#pragma once
#include <concepts>

namespace rmcs {

template <class T>
concept scalar2d_struct_trait = requires(T t) {
    { t.x } -> std::convertible_to<double>;
    { t.y } -> std::convertible_to<double>;
};
template <class T>
concept scalar2d_object_trait = requires(T t) {
    { t.x() } -> std::convertible_to<double>;
    { t.y() } -> std::convertible_to<double>;
};
template <class T>
concept scalar2d_trait = scalar2d_struct_trait<T> || scalar2d_object_trait<T>;

template <class T>
concept scalar3d_struct_trait = requires(T t) {
    { t.x } -> std::convertible_to<double>;
    { t.y } -> std::convertible_to<double>;
    { t.z } -> std::convertible_to<double>;
};
template <class T>
concept scalar3d_object_trait = requires(T t) {
    { t.x() } -> std::convertible_to<double>;
    { t.y() } -> std::convertible_to<double>;
    { t.z() } -> std::convertible_to<double>;
};
template <class T>
concept scalar3d_trait = scalar3d_struct_trait<T> || scalar3d_object_trait<T>;

template <class T>
concept scalar4d_struct_trait = requires(T t) {
    { t.x } -> std::convertible_to<double>;
    { t.y } -> std::convertible_to<double>;
    { t.z } -> std::convertible_to<double>;
    { t.w } -> std::convertible_to<double>;
};
template <class T>
concept scalar4d_object_trait = requires(T t) {
    { t.x() } -> std::convertible_to<double>;
    { t.y() } -> std::convertible_to<double>;
    { t.z() } -> std::convertible_to<double>;
    { t.w() } -> std::convertible_to<double>;
};
template <class T>
concept scalar4d_trait = scalar4d_struct_trait<T> || scalar4d_object_trait<T>;

namespace linear::details {
    template <typename Src, typename Dst>
    inline auto clone_scalar2d(const Src& src, Dst& dst) noexcept {
        if constexpr (scalar2d_object_trait<Src> && scalar2d_object_trait<Dst>) {
            dst.x() = src.x();
            dst.y() = src.y();
        } else if constexpr (scalar2d_struct_trait<Src> && scalar2d_struct_trait<Dst>) {
            dst.x = src.x;
            dst.y = src.y;
        } else if constexpr (scalar2d_object_trait<Src> && scalar2d_struct_trait<Dst>) {
            dst.x = src.x();
            dst.y = src.y();
        } else if constexpr (scalar2d_struct_trait<Src> && scalar2d_object_trait<Dst>) {
            dst.x() = src.x;
            dst.y() = src.y;
        } else {
            static_assert(false, "clone_scalar2d: unsupported trait combination");
        }
        return dst;
    }
    template <typename Src, typename Dst>
    inline auto clone_scalar3d(const Src& src, Dst& dst) noexcept {
        if constexpr (scalar3d_object_trait<Src> && scalar3d_object_trait<Dst>) {
            dst.x() = src.x();
            dst.y() = src.y();
            dst.z() = src.z();
        } else if constexpr (scalar3d_struct_trait<Src> && scalar3d_struct_trait<Dst>) {
            dst.x = src.x;
            dst.y = src.y;
            dst.z = src.z;
        } else if constexpr (scalar3d_object_trait<Src> && scalar3d_struct_trait<Dst>) {
            dst.x = src.x();
            dst.y = src.y();
            dst.z = src.z();
        } else if constexpr (scalar3d_struct_trait<Src> && scalar3d_object_trait<Dst>) {
            dst.x() = src.x;
            dst.y() = src.y;
            dst.z() = src.z;
        } else {
            static_assert(false, "clone_scalar3d: unsupported trait combination");
        }
        return dst;
    }
    template <typename Src, typename Dst>
    inline auto clone_scalar4d(const Src& src, Dst& dst) noexcept {
        if constexpr (scalar4d_object_trait<Src> && scalar4d_object_trait<Dst>) {
            dst.x() = src.x();
            dst.y() = src.y();
            dst.z() = src.z();
            dst.w() = src.w();
        } else if constexpr (scalar4d_struct_trait<Src> && scalar4d_struct_trait<Dst>) {
            dst.x = src.x;
            dst.y = src.y;
            dst.z = src.z;
            dst.w = src.w;
        } else if constexpr (scalar4d_object_trait<Src> && scalar4d_struct_trait<Dst>) {
            dst.x = src.x();
            dst.y = src.y();
            dst.z = src.z();
            dst.w = src.w();
        } else if constexpr (scalar4d_struct_trait<Src> && scalar4d_object_trait<Dst>) {
            dst.x() = src.x;
            dst.y() = src.y;
            dst.z() = src.z;
            dst.w() = src.w;
        } else {
            static_assert(false, "clone_scalar4d: unsupported trait combination");
        }
        return dst;
    }
}

struct Scalar2d {
    double x = 0;
    double y = 0;

    constexpr Scalar2d() noexcept = default;
    constexpr explicit Scalar2d(double x, double y) noexcept
        : x { x }
        , y { y } { }
    constexpr explicit Scalar2d(const scalar2d_trait auto& t) noexcept {
        linear::details::clone_scalar2d(t, *this);
    }
    auto operator=(const scalar2d_trait auto& t) noexcept -> Scalar2d& {
        linear::details::clone_scalar2d(t, *this);
        return *this;
    }
    auto copy_to(scalar2d_trait auto& target) const noexcept -> void {
        linear::details::clone_scalar2d(*this, target);
    }
    template <class T>
    auto make() const -> T {
        auto result = T { };
        return linear::details::clone_scalar2d(*this, result);
    }

    static constexpr auto kZero() { return Scalar2d { 0, 0 }; }
};
using Point2d = Scalar2d;

struct Scalar3d {
    double x = 0;
    double y = 0;
    double z = 0;

    constexpr Scalar3d() noexcept = default;
    constexpr explicit Scalar3d(double x, double y, double z) noexcept
        : x { x }
        , y { y }
        , z { z } { }
    constexpr explicit Scalar3d(const scalar3d_trait auto& t) noexcept {
        linear::details::clone_scalar3d(t, *this);
    }
    auto operator=(const scalar3d_trait auto& t) noexcept -> Scalar3d& {
        linear::details::clone_scalar3d(t, *this);
        return *this;
    }
    auto copy_to(scalar3d_trait auto& target) const noexcept -> void {
        linear::details::clone_scalar3d(*this, target);
    }
    template <class T>
    auto make() const -> T {
        auto result = T { };
        return linear::details::clone_scalar3d(*this, result);
    }

    static constexpr auto kZero() { return Scalar3d { 0, 0, 0 }; }
};
using Vector3d    = Scalar3d;
using Point3d     = Scalar3d;
using Translation = Scalar3d;
using Direction3d = Scalar3d;

struct Orientation {
    double x = 0;
    double y = 0;
    double z = 0;
    double w = 1;

    constexpr Orientation() noexcept = default;
    constexpr explicit Orientation(double x, double y, double z, double w) noexcept
        : x { x }
        , y { y }
        , z { z }
        , w { w } { }
    constexpr explicit Orientation(const scalar4d_trait auto& q) noexcept {
        linear::details::clone_scalar4d(q, *this);
    }
    auto operator=(const scalar4d_trait auto& q) noexcept -> Orientation& {
        linear::details::clone_scalar4d(q, *this);
        return *this;
    }
    auto copy_to(scalar4d_trait auto& target) const noexcept -> void {
        linear::details::clone_scalar4d(*this, target);
    }
    template <class T>
    auto make() const -> T {
        auto result = T { };
        return linear::details::clone_scalar4d(*this, result);
    }

    static constexpr auto kIdentity() { return Orientation { 0, 0, 0, 1 }; }
};

}
