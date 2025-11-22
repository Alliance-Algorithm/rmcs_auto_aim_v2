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

struct Translation {
    double x = 0;
    double y = 0;
    double z = 0;

    constexpr explicit Translation() noexcept = default;
    constexpr explicit Translation(double x, double y, double z) noexcept
        : x { x }
        , y { y }
        , z { z } { }
    constexpr explicit Translation(const translation_trait auto& t) noexcept {
        linear::details::clone_translation(t, *this);
    }
    auto operator=(const translation_trait auto& t) noexcept -> Translation& {
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

struct Orientation {
    double x = 0;
    double y = 0;
    double z = 0;
    double w = 1;

    constexpr explicit Orientation() noexcept = default;
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
