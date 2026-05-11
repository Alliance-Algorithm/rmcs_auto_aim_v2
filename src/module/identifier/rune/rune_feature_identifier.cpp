#include "rune_feature_identifier.hpp"

#include "rune_center_identifier.hpp"

namespace rmcs::identifier {

struct RuneFeatureIdentifier::Impl {
    RuneCenterIdentifier center_identifier;

    auto initialize(const YAML::Node& yaml) noexcept -> std::expected<void, std::string> {
        const auto center_yaml = yaml["center"];
        if (!center_yaml) return std::unexpected { "Missing key 'center'." };

        return center_identifier.initialize(center_yaml);
    }

    auto sync_identify(const RunePreprocessResult& input) const noexcept
        -> std::expected<RuneFeatureIdentifyResult, std::string> {
        auto centers = center_identifier.identify(input);
        if (!centers.has_value()) return std::unexpected { centers.error() };

        return RuneFeatureIdentifyResult {
            .centers = std::move(*centers),
        };
    }
};

RuneFeatureIdentifier::RuneFeatureIdentifier() noexcept
    : pimpl(std::make_unique<Impl>()) { }

RuneFeatureIdentifier::~RuneFeatureIdentifier() noexcept = default;

auto RuneFeatureIdentifier::initialize(const YAML::Node& yaml) noexcept
    -> std::expected<void, std::string> {
    return pimpl->initialize(yaml);
}

auto RuneFeatureIdentifier::sync_identify(const RunePreprocessResult& input) noexcept
    -> std::expected<RuneFeatureIdentifyResult, std::string> {
    return pimpl->sync_identify(input);
}

} // namespace rmcs::identifier
