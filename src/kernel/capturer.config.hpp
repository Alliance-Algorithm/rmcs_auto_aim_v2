#pragma once
#include "utility/serializable.hpp"

namespace rmcs::kernel {

struct CapturerConfig : util::SerializableExpansion {
    static constexpr auto kMaxGain = float { 16.9807 };

    util::integer_t timeout_ms = 2000;

    util::double_t exposure_us = 2000.;
    util::double_t framerate   = 80;
    util::double_t gain        = kMaxGain;

    util::flag_t invert_image  = false;
    util::flag_t software_sync = false;

    util::flag_t trigger_mode    = false;
    util::flag_t fixed_framerate = true;

    util::flag_t print_loss_framerate { true };
    util::integer_t print_loss_framerate_interval_seconds { 5 };

    static constexpr std::tuple metas {
        &CapturerConfig::timeout_ms,
        "timeout_ms",
        &CapturerConfig::exposure_us,
        "exposure_us",
        &CapturerConfig::framerate,
        "framerate",
        &CapturerConfig::gain,
        "gain",
        &CapturerConfig::invert_image,
        "invert_image",
        &CapturerConfig::software_sync,
        "software_sync",
        &CapturerConfig::trigger_mode,
        "trigger_mode",
        &CapturerConfig::fixed_framerate,
        "fixed_framerate",
        &CapturerConfig::print_loss_framerate,
        "print_loss_framerate",
        &CapturerConfig::print_loss_framerate_interval_seconds,
        "print_loss_framerate_interval_seconds",
    };

    template <class HikConfig>
    auto transform_to(HikConfig& to) const noexcept {
        to.timeout_ms = timeout_ms;

        to.exposure_us = static_cast<float>(exposure_us);
        to.framerate   = static_cast<float>(framerate);
        to.gain        = static_cast<float>(gain);

        to.invert_image  = invert_image;
        to.software_sync = software_sync;

        to.trigger_mode    = trigger_mode;
        to.fixed_framerate = fixed_framerate;
    }
};

}
