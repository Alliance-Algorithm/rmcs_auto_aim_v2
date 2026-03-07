#pragma once
#include "utility/string.hpp"

#include <algorithm>
#include <array>
#include <expected>
#include <ranges>
#include <stdexcept>
#include <string_view>
#include <utility>

#include <openvino/core/preprocess/pre_post_process.hpp>
#include <openvino/runtime/compiled_model.hpp>
#include <openvino/runtime/core.hpp>

namespace rmcs::util::common_model::details {

using ModelCompiler = std::function<std::shared_ptr<ov::CompiledModel>(ov::Core&)>;

template <typename T>
concept model_compiler_trait = requires {
    requires std::invocable<T, ov::Core&>;
    requires std::same_as<std::invoke_result_t<T, ov::Core&>, std::shared_ptr<ov::CompiledModel>>;
};

/// @brief:
/// POD 结构体，用于语义化地设置各个维度的值，比如：
/// ```
/// auto dimensions = Dimensions{ .W = 100, .H = 100 };
/// ```
struct Dimensions {
    using Value = ov::Dimension::value_type;

    Value N = 1;
    Value C = 3;
    Value W = 0;
    Value H = 0;

    constexpr auto at(char dimension) const -> Value {
        switch (dimension) {
        case 'N':
            return N;
        case 'C':
            return C;
        case 'W':
            return W;
        case 'H':
            return H;
        default:
            throw std::runtime_error("Wrong dimension char, valid: N, C, W, H");
        }
    }
};

///
/// @brief:
/// 模型布局，用于语义化生成 layout, shape 等数据结构
///
struct TensorLayout {
private:
    std::array<char, 5> chars { '\0', '\0', '\0', '\0', '\0' };

public:
    static constexpr auto is_valid_dimension(char dimension) noexcept {
        return dimension == 'N' || dimension == 'C' || dimension == 'W' || dimension == 'H';
    }
    template <std::ranges::input_range Range>
    static constexpr auto has_unique_dimensions(const Range& parsed) noexcept {
        auto has_unique_dimensions = true;
        for (auto it = std::ranges::begin(parsed); it != std::ranges::end(parsed); ++it) {
            if (std::ranges::find(std::next(it), std::ranges::end(parsed), *it)
                != std::ranges::end(parsed)) {
                has_unique_dimensions = false;
            }
        }
        return has_unique_dimensions;
    }
    static constexpr auto is_valid_description(std::string_view description) noexcept -> bool {
        return std::ranges::all_of(description, is_valid_dimension)
            && has_unique_dimensions(description);
    }

    template <util::StaticString description>
    static consteval auto from() noexcept -> TensorLayout {
        static_assert(description.length() == 5, "Layout description must be exactly 4 characters");

        constexpr auto parsed = std::array<char, 4> {
            description.data[0],
            description.data[1],
            description.data[2],
            description.data[3],
        };
        static_assert(std::ranges::all_of(parsed, is_valid_dimension),
            "The layout description only supports N/C/W/H");
        static_assert(has_unique_dimensions(parsed),
            "The layout description must not contain duplicate dimensions");

        return TensorLayout { std::string_view { parsed.data(), 4 } };
    }

public:
    constexpr explicit TensorLayout(std::string_view description) {
        if (description.size() == 5 && description[4] == '\0') {
            description.remove_suffix(1);
        }
        if (description.size() != 4) {
            throw std::invalid_argument { "Layout description must be exactly 4 characters" };
        }
        if (!is_valid_description(description)) {
            throw std::invalid_argument { "Invalid layout description" };
        }
        std::ranges::copy_n(description.begin(), 4, chars.begin());
    }

    constexpr auto layout() const noexcept { return ov::Layout { chars.data() }; }

    constexpr auto partial_shape(const Dimensions& dimensions) const noexcept {
        return ov::PartialShape {
            dimensions.at(chars[0]),
            dimensions.at(chars[1]),
            dimensions.at(chars[2]),
            dimensions.at(chars[3]),
        };
    }
    constexpr auto shape(const Dimensions& dimensions) const noexcept {
        return ov::Shape { {
            static_cast<std::size_t>(dimensions.at(chars[0])),
            static_cast<std::size_t>(dimensions.at(chars[1])),
            static_cast<std::size_t>(dimensions.at(chars[2])),
            static_cast<std::size_t>(dimensions.at(chars[3])),
        } };
    }
};

}
namespace rmcs::util {

/// Filename: ../../develop_ws/RobotDetectionModel/Model/0526.onnx
/// ==== Model Basic Status ====
/// Model loaded: ../../develop_ws/RobotDetectionModel/Model/0526.onnx
/// Input count : 1
/// Output count: 1
/// ---- Inputs ----
/// Name        : images
/// Element type: f16
/// Shape       : [1,3,640,640]
/// Static shape: yes
/// Layout      : <not set>
/// Color format: <not stored in model metadata>
///
/// ---- Outputs ----
/// Name        : output
/// Element type: f32
/// Shape       : [1,25200,22]

/// Filename: ../models/yolov5.xml
/// ==== Model Basic Status ====
/// Model loaded: ../models/yolov5.xml
/// Input count : 1
/// Output count: 1
/// ---- Inputs ----
/// Name        : images
/// Element type: f32
/// Shape       : [1,3,640,640]
/// Static shape: yes
/// Layout      : <not set>
/// Color format: <not stored in model metadata>
///
/// ---- Outputs ----
/// Name        : output
/// Element type: f32
/// Shape       : [1,25200,22]

struct OvModelAdapter {
    std::shared_ptr<ov::CompiledModel> compiled_model = nullptr;

    // @NOTE:
    //  由于模型的构建参数并不能完全从序列化文本中传递，
    //  所幸直接用代码注册，并指定一个固定的 model 目录
    bool has_set_location = false;
    auto set_location(std::string_view location) {
        std::ignore      = location;
        has_set_location = true;
    }

    bool has_set_compiler = false;

    using ModelCompiler = common_model::details::ModelCompiler;
    ModelCompiler compiler {};

    template <common_model::details::model_compiler_trait Compiler>
    auto set_compiler(Compiler&& _compiler) noexcept {
        compiler = std::forward<Compiler>(_compiler);
    }

    auto compile_model(ov::Core& core) noexcept -> std::expected<void, std::string> {
        const auto completed = std::ranges::all_of(
            std::array {
                has_set_location,
                has_set_compiler,
            },
            std::identity {});
        if (!completed) {
            return std::unexpected { "Not set completely, check your initialization" };
        }

        try {
            compiled_model = compiler(core);
        } catch (const std::exception& e) {
            return std::unexpected { e.what() };
        }
        return {};
    }
};

struct OvModel {
    using TensorLayout = common_model::details::TensorLayout;
    using Dimensions   = common_model::details::Dimensions;

    struct Config {
        // Image
        std::string image_element_type;
        std::string image_color_format;

        // Model
        std::string location;
        std::string input_layout_str;
        std::string infer_layout_str;
        std::string infer_color_format;

        auto is_valid() const noexcept -> std::expected<void, std::string> {
            // Image
            std::array invalid_type { "u8", "u16", "f32", "f16" };
            if (!std::ranges::contains(invalid_type, image_element_type))
                return std::unexpected { std::format("{} is a wrong type", image_element_type) };

            std::array invalid_format { "bgr", "rgb", "bgrx", "rgbx", "gray" };
            if (!std::ranges::contains(invalid_format, image_color_format))
                return std::unexpected { std::format("{} is a wrong format", image_color_format) };

            // Model
            if (!std::filesystem::exists(location))
                return std::unexpected { std::format("{} is not exists in filesystem", location) };

            if (!TensorLayout::is_valid_description(input_layout_str))
                return std::unexpected { std::format(
                    "{} is a wrong input layout", input_layout_str) };
            if (!TensorLayout::is_valid_description(infer_layout_str))
                return std::unexpected { std::format(
                    "{} is a wrong infer layout", infer_layout_str) };

            if (!std::ranges::contains(invalid_format, infer_color_format))
                return std::unexpected { std::format("{} is a wrong format", image_color_format) };

            return {};
        }
    };
    Config config;

    TensorLayout input_layout;
    TensorLayout infer_layout;

    explicit OvModel(Config config, ov::Core& core)
        : config { std::move(config) }
        , input_layout { config.input_layout_str }
        , infer_layout { config.infer_layout_str } {

        using namespace common_model::details;
        auto raw_model = core.read_model(config.location);
    }
};
}
