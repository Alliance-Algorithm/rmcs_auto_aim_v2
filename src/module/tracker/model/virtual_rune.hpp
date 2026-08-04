#pragma once

#include "utility/clock.hpp"
#include "utility/math/linear.hpp"
#include "utility/pimpl.hpp"
#include "utility/robot/rune.hpp"
#include "utility/serializable.hpp"

#include <array>
#include <span>

namespace rmcs {

class VirtualRuneModel {
    RMCS_PIMPL_DEFINITION(VirtualRuneModel)

public:
    struct Config : util::Serializable {
        bool enable = false;
        bool large  = false; // false=小符（恒速 π/3），true=大符（正弦）

        // 符中心，固定 OdomLink 坐标
        double x = 6.670;
        double y = 0.0;
        double z = 2.172;

        double face_yaw = 0.0; // 符面法线（局部 +x 背面）远离相机，R 标突出朝向相机

        // 相机内参：不参与序列化，由 Tracker 拷贝填充
        std::array<double, 9> camera_matrix = { };
        std::array<double, 5> distort_coeff = { };

        static constexpr std::tuple metas {
            // clang-format off
            &Config::enable,   "enable",
            &Config::large,    "large",
            &Config::x,        "x",
            &Config::y,        "y",
            &Config::z,        "z",
            &Config::face_yaw, "face_yaw",
            // clang-format on
        };
    };

    explicit VirtualRuneModel(const Config&) noexcept;

    auto update_camera(const std::array<double, 9>&) noexcept -> void;
    auto update_camera(const std::array<double, 5>&) noexcept -> void;
    auto update_transform(const Transform&) noexcept -> void;

    auto update(Timestamp) noexcept -> void;

    auto icons() const noexcept -> std::span<const RuneIcon>;
    auto bullseyes() const noexcept -> std::span<const RuneBullseye>;
};

}
