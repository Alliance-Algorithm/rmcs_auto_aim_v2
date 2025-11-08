#include "model.hpp"
#include "utility/image.details.hpp"
#include "utility/serializable.hpp"

#include <opencv2/imgproc.hpp>

#include <openvino/core/preprocess/pre_post_process.hpp>
#include <openvino/runtime/compiled_model.hpp>
#include <openvino/runtime/core.hpp>
#include <openvino/runtime/exception.hpp>

using namespace rmcs::identifier;
using do_not_warning = rmcs::Image::Details;

struct OpenVinoNet::Impl {

    ov::CompiledModel openvino_model;
    ov::Core openvino_core;

    struct Nothing { };
    std::shared_ptr<Nothing> living_flag {
        std::make_shared<Nothing>(),
    };

    cv::Mat resized_input_buffer;

    struct NetworkConfig : util::Serializable {
        std::string model_location;
        std::string infer_device;

        bool use_roi_segment;
        bool use_corner_correction;

        int roi_w;
        int roi_h;

        double threshold;
        double min_confidence;

        int tensor_w;
        int tensor_h;

        constexpr static std::tuple metas {
            &NetworkConfig::model_location,
            "model_location",
            &NetworkConfig::infer_device,
            "infer_device",
        };
    } network_config;

    auto configure(const YAML::Node& yaml) noexcept { return network_config.serialize(yaml); }

    auto compile_openvino_model() noexcept -> std::expected<void, std::string> try {
        auto origin_model = openvino_core.read_model(network_config.model_location);
        if (!origin_model) {
            return std::unexpected { "Empty model resource was loaded from openvino core" };
        }

        auto preprocess = ov::preprocess::PrePostProcessor { origin_model };

        auto& input = preprocess.input();
        input.tensor()
            .set_element_type(ov::element::u8)
            .set_shape({ 1, 640, 640, 3 })
            .set_layout("NHWC")
            .set_color_format(ov::preprocess::ColorFormat::BGR);
        input.preprocess()
            .convert_element_type(ov::element::f32)
            .convert_color(ov::preprocess::ColorFormat::RGB)
            .scale(255.0);
        input.model().set_layout("NCHW");

        // For real-time process, use this mode
        const auto performance  = ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY);
        const auto processe_out = preprocess.build();
        openvino_model =
            openvino_core.compile_model(processe_out, network_config.infer_device, performance);
        return {};

    } catch (const std::runtime_error& e) {
        return std::unexpected { std::string { "Failed to load model | " } + e.what() };

    } catch (...) {
        return std::unexpected { "Failed to load model caused by unknown exception" };
    }

    auto generate_openvino_request() noexcept {
        auto tensor  = ov::Tensor {};
        auto request = openvino_model.create_infer_request();
    }

    auto async_infer(const Image& image, Callback callback) noexcept -> void {
        const auto& origin_mat = image.details().mat;
        if (origin_mat.empty()) [[unlikely]] {
            callback(std::unexpected { "Empty image mat" });
        }

        auto segmentation = origin_mat;
        if (network_config.use_roi_segment) {
            const auto w = network_config.roi_w;
            const auto h = network_config.roi_h;

            auto action_success = false;
            do {
                if (w > origin_mat.cols) break;
                if (h > origin_mat.rows) break;

                // Find roi corner
                const auto x    = (origin_mat.cols - w) / 2;
                const auto y    = (origin_mat.rows - h) / 2;
                const auto rect = cv::Rect2i { x, y, w, h };

                action_success = true;
                segmentation   = origin_mat(rect);
            } while (false);

            if (!action_success) {
                callback(std::unexpected { "Failed to segment image" });
            }
        }

        auto input_tensor = ov::Tensor {};
        {
            const auto w = network_config.tensor_w;
            const auto h = network_config.tensor_h;

            const auto scale = std::min(static_cast<double>(h) / segmentation.rows,
                static_cast<double>(w) / segmentation.cols);

            const auto scaled_w = static_cast<int>(segmentation.cols * scale);
            const auto scaled_h = static_cast<int>(segmentation.rows * scale);

            auto input_roi = cv::Rect2i { 0, 0, scaled_w, scaled_h };
            auto input_mat = cv::Mat { h, w, CV_8UC3, { 0, 0, 0 } };
            cv::resize(segmentation, input_mat(input_roi), { scaled_w, scaled_h });

            input_tensor = ov::Tensor { ov::element::u8,
                { 1, static_cast<std::size_t>(w), static_cast<std::size_t>(h), 3 },
                input_mat.data };
        }

        const auto tensor = ov::Tensor {
            openvino_model.input().get_element_type(),
            openvino_model.input().get_shape(),
            resized_input_buffer.data,
        };
        auto request = openvino_model.create_infer_request();
        request.set_input_tensor(tensor);

        auto living_weak = std::weak_ptr { living_flag };
        request.set_callback([=, f = std::move(callback)](const auto& e) mutable {
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
            f(std::unexpected { "" });
        });
        request.start_async();
    }
};

auto OpenVinoNet::configure(const YAML::Node& yaml) noexcept -> std::expected<void, std::string> {
    return pimpl->configure(yaml);
}

auto OpenVinoNet::sync_infer(const Image&) const noexcept -> std::vector<Armor> { }

auto OpenVinoNet::async_infer(const Image& image, Callback callback) noexcept -> void {
    return pimpl->async_infer(image, std::move(callback));
}

OpenVinoNet::OpenVinoNet() noexcept
    : pimpl { std::make_unique<Impl>() } { }

OpenVinoNet::~OpenVinoNet() noexcept = default;
