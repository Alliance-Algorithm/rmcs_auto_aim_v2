#include "model.hpp"

#include <opencv2/imgproc.hpp>

#include <openvino/core/preprocess/pre_post_process.hpp>
#include <openvino/runtime/compiled_model.hpp>
#include <openvino/runtime/core.hpp>
#include <openvino/runtime/exception.hpp>

using namespace rmcs::identifier;

constexpr auto kPreElementType = ov::element::u8;
constexpr auto kPreLayout      = "NHWC";
constexpr auto kPreColorFormat = ov::preprocess::ColorFormat::BGR;

constexpr auto kPreConvertElementType = ov::element::f32;
constexpr auto kPreConvertColor       = ov::preprocess::ColorFormat::RGB;

const auto kPreScale = std::vector<float> { 255., 255., 255. };

constexpr auto kPreModelLayout       = "NCHW";
constexpr auto kPreTensorElementType = ov::element::f32;

constexpr auto kCompileDevice = "AUTO";

struct NeuralNetwork::Impl {
    struct Nothing { };
    std::shared_ptr<Nothing> living_flag = std::make_shared<Nothing>();

    Config config;
    ov::CompiledModel openvino_model;

    cv::Mat resized_input_buffer;

    auto make_configuration(const Config& config) noexcept -> std::expected<void, std::string> {
        if (config.color == CampColor::UNKNOWN) {
            return std::unexpected { "Illegal camp color was set while configuring" };
        }
        this->config = config;
        return {};
    }

    auto load_from_filesystem(const std::string& location) noexcept
        -> std::expected<void, std::string> try {

        auto openvino_core = ov::Core {};
        auto origin_model  = openvino_core.read_model(location);
        if (!origin_model) {
            return std::unexpected { "Empty model resource was loaded from openvino core" };
        }
        {
            auto preprocessor = ov::preprocess::PrePostProcessor { origin_model };

            auto& input = preprocessor.input();
            input.tensor()
                .set_element_type(kPreElementType)
                .set_layout(kPreLayout)

                .set_color_format(kPreColorFormat);
            input.preprocess()
                .convert_element_type(kPreConvertElementType)
                .convert_color(kPreConvertColor)
                .scale(kPreScale);
            input.model().set_layout(kPreModelLayout);

            auto& output = preprocessor.output();
            output.tensor().set_element_type(kPreTensorElementType);

            openvino_model = openvino_core.compile_model(preprocessor.build(), kCompileDevice);
        }
        return {};

    } catch (std::runtime_error const& e) {
        return std::unexpected { std::string { "Failed to load model: " } + e.what() };

    } catch (...) {
        return std::unexpected { "Failed to load model caused by unknown exception" };
    }

    // TODO: To implement
    auto request_inference(const Input& input) noexcept -> void {

        cv::resize(input, resized_input_buffer, config.infer_size);

        const auto tensor = ov::Tensor {
            openvino_model.input().get_element_type(),
            openvino_model.input().get_shape(),
            resized_input_buffer.data,
        };
        auto request = openvino_model.create_infer_request();
        request.set_input_tensor(tensor);

        auto living_weak = std::weak_ptr { living_flag };
        request.set_callback([living_weak](const std::exception_ptr& exception) mutable {
            if (!living_weak.lock()) {
                return;
            }
            if (exception) {
                try {
                    std::rethrow_exception(exception);
                } catch (const ov::Cancelled& e) {
                } catch (const ov::Busy& e) {
                } catch (const std::exception& e) { }
            }
        });
        request.start_async();
    }
};

NeuralNetwork::NeuralNetwork() noexcept
    : pimpl { std::make_unique<Impl>() } { }

NeuralNetwork::~NeuralNetwork() noexcept = default;
