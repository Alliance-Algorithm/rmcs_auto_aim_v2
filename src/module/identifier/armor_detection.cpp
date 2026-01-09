#include "armor_detection.hpp"
#include "module/identifier/model.hpp"
#include "utility/robot/id.hpp"

using namespace rmcs::identifier;

struct ArmorDetection::Impl {
    OpenVinoNet openvino_net;

    auto initialize(const YAML::Node& yaml) noexcept {
        return openvino_net.configure(yaml); //
    }
    auto sync_detect(const Image& image) const noexcept -> std::optional<std::vector<Armor2D>> {
        auto result = openvino_net.sync_infer(image);
        if (!result.has_value()) {
            return std::nullopt;
        }
        const auto& raw_results = result.value();

        auto result_armors = std::vector<Armor2D> {};
        for (const auto& raw_armor : raw_results) {
            auto armor  = Armor2D {};
            armor.genre = raw_armor.armor_genre();
            armor.color = raw_armor.armor_color();

            armor.tl = raw_armor.tl();
            armor.tr = raw_armor.tr();
            armor.bl = raw_armor.bl();
            armor.br = raw_armor.br();

            armor.confidence = raw_armor.confidence;

            constexpr auto devices = DeviceIds::kLargeArmor();
            armor.shape = devices.contains(armor.genre) ? ArmorShape::LARGE : ArmorShape::SMALL;

            result_armors.push_back(armor);
        }
        return result_armors;
    }
};

auto ArmorDetection::initialize(const YAML::Node& yaml) noexcept
    -> std::expected<void, std::string> {
    return pimpl->initialize(yaml);
}

auto ArmorDetection::sync_detect(const Image& image) noexcept
    -> std::optional<std::vector<Armor2D>> {
    return pimpl->sync_detect(image);
}

ArmorDetection::ArmorDetection() noexcept
    : pimpl { std::make_unique<Impl>() } { }

ArmorDetection::~ArmorDetection() noexcept = default;
