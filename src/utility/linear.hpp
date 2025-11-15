#pragma once

namespace rmcs {

template <class T>
concept translation_trait = requires(T t) {
    t.x();
    t.y();
    t.z();
};
template <class Q>
concept orientation_trait = requires(Q q) {
    q.x();
    q.y();
    q.z();
    q.w();
};

struct Translation {
    double x;
    double y;
    double z;
    constexpr explicit Translation() noexcept = default;
    constexpr explicit Translation(const translation_trait auto& t) noexcept
        : x { t.x() }
        , y { t.y() }
        , z { t.z() } { }
    auto operator=(const translation_trait auto& t) noexcept -> Translation& {
        x = t.x();
        y = t.y();
        z = t.z();
        return *this;
    }
    auto copy_to(auto& target) const noexcept -> void
        requires requires { target.x; }
    {
        target.x = x;
        target.y = y;
        target.z = z;
    }
    auto copy_to(auto& target) const noexcept -> void
        requires requires { target.x(); }
    {
        target.x() = x;
        target.y() = y;
        target.z() = z;
    }

    template <class T>
    auto make() -> T {
        auto result = T {};
        copy_to(result);
        return result;
    }
};
struct Orientation {
    double x;
    double y;
    double z;
    double w;
    constexpr explicit Orientation() noexcept = default;
    constexpr explicit Orientation(const orientation_trait auto& q) noexcept
        : x { q.x() }
        , y { q.y() }
        , z { q.z() }
        , w { q.w() } { }
    auto operator=(const orientation_trait auto& q) noexcept -> Orientation& {
        x = q.x();
        y = q.y();
        z = q.z();
        w = q.w();
        return *this;
    }
    auto copy_to(auto& target) const noexcept -> void
        requires requires { target.x; }
    {
        target.x = x;
        target.y = y;
        target.z = z;
        target.w = w;
    }
    auto copy_to(auto& target) const noexcept -> void
        requires requires { target.x(); }
    {
        target.x() = x;
        target.y() = y;
        target.z() = z;
        target.w() = w;
    }

    template <class T>
    auto make() -> T {
        auto result = T {};
        copy_to(result);
        return result;
    }
};

}
