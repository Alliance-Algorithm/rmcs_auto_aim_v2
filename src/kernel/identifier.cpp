#include "identifier.hpp"
#include "module/identifier/model.hpp"
#include "utility/robot/armor.hpp"
#include "utility/serializable.hpp"

using namespace rmcs;
using namespace rmcs::kernel;

struct Detector {
    virtual ~Detector() = default;
    virtual auto exec(const Image& src) -> void {
        //
    }
};

struct Config : util::Serializable {
    std::string model_location;
    std::string infer_device;

    bool use_roi_segment;
    bool use_corner_correction;

    int roi_rows;
    int roi_cols;
    int input_rows;
    int input_cols;

    double min_confidence;
    double score_threshold;
    double nms_threshold;

    constexpr static std::tuple metas {
        // clang-format off
        &Config::model_location,        "model_location",
        &Config::infer_device,          "infer_device",
        &Config::use_roi_segment,       "use_roi_segment",
        &Config::use_corner_correction, "use_corner_correction",
        &Config::roi_rows,              "roi_rows",
        &Config::roi_cols,              "roi_cols",
        &Config::input_rows,            "input_rows",
        &Config::input_cols,            "input_cols",
        &Config::min_confidence,        "min_confidence",
        &Config::score_threshold,       "score_threshold",
        &Config::nms_threshold,         "nms_threshold",
        // clang-format on
    };
};

struct Identifier::Impl {

    auto initialize(const YAML::Node&) noexcept -> std::expected<void, std::string> {
        return std::unexpected { "Not implemented" };
    }

    auto identify(const Image&) noexcept {
        // ...
    }
};

auto Identifier::initialize(const YAML::Node& yaml) noexcept -> std::expected<void, std::string> {
    return pimpl->initialize(yaml);
}

auto Identifier::perview() const noexcept -> const Image& { }

auto Identifier::sync_identify(const Image&) noexcept -> void { }

Identifier::Identifier() noexcept
    : pimpl { std::make_unique<Impl>() } { }

Identifier::~Identifier() noexcept = default;
