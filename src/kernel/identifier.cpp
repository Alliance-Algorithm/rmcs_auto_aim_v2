#include "identifier.hpp"

#include "modules/identifier/armor.hpp"
#include "modules/identifier/lightbar.hpp"
#include "modules/identifier/preprocess.hpp"

using namespace rmcs::runtime;

struct Identifier::Impl {
    using ImageUnique = std::unique_ptr<Image>;

    PreProcess preprocess;
    LightbarFinder lightbar_finder;
    ArmorFinder armor_finder;

    auto identify(const Image& image) noexcept {
        // ...
    }
};

auto Identifier::initialize(const Config&) noexcept -> std::expected<void, std::string> { }

auto Identifier::perview() const noexcept -> const Image& { }

auto Identifier::sync_identify(const Image&) noexcept -> void { }

Identifier::Identifier() noexcept
    : pimpl { std::make_unique<Impl>() } { }

Identifier::~Identifier() noexcept = default;
