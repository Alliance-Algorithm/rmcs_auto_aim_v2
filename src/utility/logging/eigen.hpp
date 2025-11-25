#pragma once
#include <eigen3/Eigen/Geometry>
#include <format>

struct EigenFormatterBase {
    int precision = -1;

    constexpr auto parse(std::format_parse_context& ctx) {
        auto it = ctx.begin();
        if (it != ctx.end() && *it == '.') {
            ++it;
            precision = 0;
            while (it != ctx.end() && (*it >= '0' && *it <= '9')) {
                precision = precision * 10 + (*it - '0');
                ++it;
            }
        }
        return it;
    }
};

template <>
struct std::formatter<Eigen::Vector3d> : EigenFormatterBase {
    auto format(const Eigen::Vector3d& v, std::format_context& ctx) const {
        if (precision >= 0) {
            return std::format_to(ctx.out(), "({:.{}f}, {:.{}f}, {:.{}f})", v.x(), precision, v.y(),
                precision, v.z(), precision);
        }
        return std::format_to(ctx.out(), "({}, {}, {})", v.x(), v.y(), v.z());
    }
};

template <>
struct std::formatter<Eigen::Quaterniond> : EigenFormatterBase {
    auto format(const Eigen::Quaterniond& q, std::format_context& ctx) const {
        if (precision >= 0) {
            return std::format_to(ctx.out(), "(w={:.{}f}, {:.{}f}, {:.{}f}, {:.{}f})", q.w(),
                precision, q.x(), precision, q.y(), precision, q.z(), precision);
        }
        return std::format_to(ctx.out(), "(w={}, {}, {}, {})", q.w(), q.x(), q.y(), q.z());
    }
};

template <>
struct std::formatter<Eigen::AngleAxisd> : EigenFormatterBase {
    auto format(const Eigen::AngleAxisd& aa, std::format_context& ctx) const {
        const auto& axis = aa.axis();
        if (precision >= 0) {
            return std::format_to(ctx.out(), "(angle={:.{}f}, axis=({:.{}f}, {:.{}f}, {:.{}f}))",
                aa.angle(), precision, axis.x(), precision, axis.y(), precision, axis.z(),
                precision);
        }
        return std::format_to(
            ctx.out(), "(angle={}, axis=({}, {}, {}))", aa.angle(), axis.x(), axis.y(), axis.z());
    }
};

template <>
struct std::formatter<Eigen::Isometry3d> : EigenFormatterBase {
    auto format(const Eigen::Isometry3d& iso, std::format_context& ctx) const {
        const auto& t = Eigen::Vector3d { iso.translation() };
        const auto& q = Eigen::Quaterniond { iso.rotation() };

        if (precision >= 0) {
            return std::format_to(ctx.out(),
                "T: ({:.{}f}, {:.{}f}, {:.{}f}), Q: (w={:.{}f}, {:.{}f}, {:.{}f}, {:.{}f})", t.x(),
                precision, t.y(), precision, t.z(), precision, q.w(), precision, q.x(), precision,
                q.y(), precision, q.z(), precision);
        }
        return std::format_to(ctx.out(), "T: ({}, {}, {}), Q: (w={}, {}, {}, {})", t.x(), t.y(),
            t.z(), q.w(), q.x(), q.y(), q.z());
    }
};
