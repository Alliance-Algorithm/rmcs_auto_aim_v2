#pragma once

#include "module/identifier/rune/rune_preprocessor.hpp"
#include "module/identifier/rune/rune_types.hpp"
#include "utility/pimpl.hpp"

#include <expected>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace rmcs::identifier {

class RuneCenterIdentifier {
    RMCS_PIMPL_DEFINITION(RuneCenterIdentifier)

public:
    auto initialize(const YAML::Node&) noexcept -> std::expected<void, std::string>;

    auto identify(const RunePreprocessResult&) const noexcept
        -> std::expected<std::vector<RuneCenterCandidate>, std::string>;
};

} // namespace rmcs::identifier
