#include "model.hpp"
#include "utility/image.details.hpp"

#include <opencv2/imgproc.hpp>

#include <openvino/core/preprocess/pre_post_process.hpp>
#include <openvino/runtime/compiled_model.hpp>
#include <openvino/runtime/core.hpp>
#include <openvino/runtime/exception.hpp>

using namespace rmcs::identifier;
using do_not_warning = rmcs::Image::Details;

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

    auto async_infer(const Image& image, std::coroutine_handle<> h) noexcept -> void {

        cv::resize(image.details().mat, resized_input_buffer, config.infer_size);

        const auto tensor = ov::Tensor {
            openvino_model.input().get_element_type(),
            openvino_model.input().get_shape(),
            resized_input_buffer.data,
        };
        auto request = openvino_model.create_infer_request();
        request.set_input_tensor(tensor);

        auto living_weak = std::weak_ptr { living_flag };
        request.set_callback([=](const auto& e) mutable {
            if (!living_weak.lock()) {
                return;
            }
            if (e) {
                try {
                    std::rethrow_exception(e);
                } catch (const ov::Cancelled& e) {
                } catch (const ov::Busy& e) {
                } catch (const std::exception& e) { }
            }

            auto output = request.get_output_tensor();
        });
        request.start_async();
    }
};

auto NeuralNetwork::configure(const Config& config) noexcept -> std::expected<void, std::string> {
    return pimpl->make_configuration(config);
}

auto NeuralNetwork::load_from_filesystem(const std::string& location) noexcept
    -> std::expected<void, std::string_view> {
    return pimpl->load_from_filesystem(location);
}

auto NeuralNetwork::sync_infer(const Image&) const noexcept -> std::vector<Armor> { }

auto NeuralNetwork::async_infer(std::shared_ptr<Image const> image, Callback callback) noexcept
    -> void { }

NeuralNetwork::NeuralNetwork() noexcept
    : pimpl { std::make_unique<Impl>() } { }

NeuralNetwork::~NeuralNetwork() noexcept = default;
