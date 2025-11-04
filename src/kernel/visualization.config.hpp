#pragma once
#include "utility/serializable.hpp"

namespace rmcs::runtime {

constexpr std::array kVideoTypes {
    "RTP_JEPG",
    "RTP_H264",
};

struct VisualizationConfig : util::Serializable {

    util::integer_t framerate = 80;

    util::string_t monitor_host = "localhost";
    util::string_t monitor_port = "5000";

    util::string_t stream_type = "RTP_JEPG";

    static constexpr auto metas = std::tuple {
        &VisualizationConfig::framerate,
        "framerate",
        &VisualizationConfig::monitor_host,
        "monitor_host",
        &VisualizationConfig::monitor_port,
        "monitor_port",
        &VisualizationConfig::stream_type,
        "stream_type",
    };
};

}
