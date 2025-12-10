#pragma once
#include "utility/image/image.hpp"
#include "utility/serializable.hpp"
#include <expected>
#include <yaml-cpp/yaml.h>

namespace rmcs::cap {

struct LocalVideo {
    RMCS_PIMPL_DEFINITION(LocalVideo)

private:
    struct ConfigDetail {
        std::string location;
        double frame_rate;
        bool loop_play;
        bool allow_skipping;
    };

public:
    struct Config : ConfigDetail, util::Serializable {
        constexpr static std::tuple metas {
            &ConfigDetail::location,
            "location", // 视频文件路径
            &ConfigDetail::frame_rate,
            "frame_rate", // 帧率
            &ConfigDetail::loop_play,
            "loop_play", // 循环播放
            &ConfigDetail::allow_skipping,
            "allow_skipping" // 允许跳帧以保证实时性
        };
    };

    auto configure(Config const&) -> std::expected<void, std::string>;

    auto connect() noexcept -> std::expected<void, std::string>;

    auto connected() const noexcept -> bool;

    auto disconnect() noexcept -> std::expected<void, std::string>;

    auto wait_image() noexcept -> std::expected<std::unique_ptr<Image>, std::string>;
};
}
