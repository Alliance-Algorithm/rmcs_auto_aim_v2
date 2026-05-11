#pragma once

#include "module/identifier/rune/rune_preprocessor.hpp"
#include "module/identifier/rune/rune_types.hpp"
#include "utility/pimpl.hpp"

#include <expected>
#include <string>

#include <yaml-cpp/yaml.h>

namespace rmcs::identifier {

class RuneFeatureIdentifier {
    RMCS_PIMPL_DEFINITION(RuneFeatureIdentifier)

public:
    auto initialize(const YAML::Node&) noexcept -> std::expected<void, std::string>;

    auto sync_identify(const RunePreprocessResult&) noexcept
        -> std::expected<RuneFeatureIdentifyResult, std::string>;
};

}
