#include "identifier.hpp"
#include "module/identifier/armor_detection.hpp"
#include "utility/robot/armor.hpp"

using namespace rmcs;
using namespace rmcs::kernel;
using namespace rmcs::identifier;

struct Identifier::Impl {
    ArmorDetection armor_detection;

    auto initialize(const YAML::Node& yaml) noexcept -> std::expected<void, std::string> {
        {
            auto result = armor_detection.initialize(yaml);
            if (!result.has_value()) {
                return std::unexpected { result.error() };
            }
        }

        return {};
    }

    auto identify(const Image& src) noexcept { return armor_detection.sync_detect(src); }
};

auto Identifier::initialize(const YAML::Node& yaml) noexcept -> std::expected<void, std::string> {
    return pimpl->initialize(yaml);
}

auto Identifier::sync_identify(const Image& src) noexcept -> std::optional<std::vector<Armor2D>> {
    return pimpl->identify(src);
}

Identifier::Identifier() noexcept
    : pimpl { std::make_unique<Impl>() } { }

Identifier::~Identifier() noexcept = default;
