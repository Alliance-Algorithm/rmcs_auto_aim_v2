#include "armor_detection.hpp"

#include "module/identifier/models/shenzhen_0526.hpp"
#include "module/identifier/models/shenzhen_0708.hpp"
#include "module/identifier/models/tongji_yolov5.hpp"

#include "utility/image/image.details.hpp"
#include "utility/math/sigmoid.hpp"
#include "utility/model/common_model.hpp"
#include "utility/robot/id.hpp"
#include "utility/serializable.hpp"

#include <filesystem>

#include <opencv2/dnn/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <openvino/runtime/compiled_model.hpp>
#include <openvino/runtime/core.hpp>
#include <openvino/runtime/exception.hpp>

using namespace rmcs::identifier;
using do_not_warning = rmcs::Image::Details;

struct ArmorDetection::Impl {

    struct ExplainInterface {
        virtual ~ExplainInterface() = default;

        virtual auto explain(ov::InferRequest&) const noexcept -> Armor2Ds = 0;
    };

    struct Config : util::Serializable {
        std::string model_location;
        std::string infer_device;

        bool use_roi_segment;

        int roi_rows;
        int roi_cols;

        int input_rows;
        int input_cols;

        float min_confidence;

        float score_threshold;
        float nms_threshold;

        constexpr static std::tuple metas {
            // clang-format off
            &Config::model_location,         "model_location",
            &Config::infer_device,           "infer_device",
            &Config::use_roi_segment,        "use_roi_segment",
            &Config::roi_rows,               "roi_rows",
            &Config::roi_cols,               "roi_cols",
            &Config::input_rows,             "input_rows",
            &Config::input_cols,             "input_cols",
            &Config::min_confidence,         "min_confidence",
            &Config::score_threshold,        "score_threshold",
            &Config::nms_threshold,          "nms_threshold",
            // clang-format on
        };
    } config;

    template <class model_type>
    struct StaticExplainFunctor final : ExplainInterface {
        float min_confidence;
        float score_threshold;
        float nms_threshold;
        const float& adapt_scaling;
        const cv::Point2f& roi_offset;

        explicit StaticExplainFunctor(float min_confidence, float score_threshold,
            float nms_threshold, const float& scaling, const cv::Point2f& offset) noexcept
            : min_confidence { min_confidence }
            , score_threshold { score_threshold }
            , nms_threshold { nms_threshold }
            , adapt_scaling { scaling }
            , roi_offset { offset } { }

        auto explain(ov::InferRequest& finished_request) const noexcept -> Armor2Ds override {
            using result_type    = typename model_type::Result;
            using precision_type = typename result_type::precision_type;

            auto tensor = finished_request.get_output_tensor();
            auto& shape = tensor.get_shape();

            const auto rows = static_cast<std::size_t>(shape.at(1));
            const auto cols = static_cast<std::size_t>(shape.at(2));
            if (cols != result_type::length()) {
                return {};
            }

            auto parsed_results = std::vector<result_type> {};
            auto scores         = std::vector<float> {};
            auto boxes          = std::vector<cv::Rect> {};

            const auto* data = tensor.data<precision_type>();
            for (std::size_t row = 0; row < rows; row++) {
                auto line = result_type {};
                line.unsafe_from(std::span { data + row * cols, cols });
                line.confidence() = util::sigmoid(line.confidence());

                if (line.confidence() > min_confidence) {
                    parsed_results.push_back(line);
                    scores.push_back(line.confidence());
                    boxes.push_back(line.bounding_rect());
                }
            }

            auto kept_points = std::vector<int> {};
            cv::dnn::NMSBoxes(boxes, scores, score_threshold, nms_threshold, kept_points);

            auto final_result = Armor2Ds {};
            final_result.reserve(kept_points.size());

            for (auto idx : kept_points) {
                auto armor       = cast_to_armor_result(parsed_results[idx]);
                const auto scale = 1.f / adapt_scaling;
                for (auto* point : { &armor.tl, &armor.tr, &armor.br, &armor.bl }) {
                    *point = (*point) * scale + roi_offset;
                }

                armor.center = (armor.tl + armor.tr + armor.br + armor.bl) * 0.25f;
                final_result.push_back(armor);
            }
            return final_result;
        }
    };

    struct Nothing { };
    std::shared_ptr<Nothing> living_flag {
        std::make_shared<Nothing>(),
    };

    ov::Core openvino_core;
    ov::CompiledModel openvino_model;
    std::unique_ptr<ExplainInterface> explain_infer_functor;

    TensorLayout input_layout = TensorLayout::from<"NHWC">();
    Dimensions input_dimensions { .W = 640, .H = 640 };

    float adapt_scaling = 1.f;
    cv::Point2f roi_offset { 0.f, 0.f };

    auto configure(const YAML::Node& yaml) noexcept -> std::expected<void, std::string> {
        auto result = config.serialize(yaml);
        if (!result.has_value()) {
            return std::unexpected { result.error() };
        }

        try {
            const auto model_name =
                std::filesystem::path { config.model_location }.filename().string();

            /*''*/ if (model_name == TongJiYoloV5::kLocation) {
                compile_model_with<TongJiYoloV5>();
            } else if (model_name == ShenZhen0526::kLocation) {
                compile_model_with<ShenZhen0526>();
            } else if (model_name == ShenZhen0708::kLocation) {
                compile_model_with<ShenZhen0708>();
            } else {
                return std::unexpected { "Unsupported model type: " + model_name };
            }
            return {};

        } catch (const std::runtime_error& e) {
            return std::unexpected { std::string { "Failed to load model | " } + e.what() };

        } catch (...) {
            return std::unexpected { "Failed to load model caused by unknown exception" };
        }
    }

    template <class model_type>
    auto compile_model_with() -> void {
        auto model = model_type {};
        if (!config.infer_device.empty()) {
            model.infer_device = config.infer_device;
        }

        if (config.input_cols > 0) {
            model.dimensions.W = config.input_cols;
        }
        if (config.input_rows > 0) {
            model.dimensions.H = config.input_rows;
        }

        openvino_model   = model.compile(openvino_core, config.model_location);
        input_layout     = model.input_layout;
        input_dimensions = model.dimensions;
        explain_infer_functor =
            std::make_unique<StaticExplainFunctor<model_type>>(config.min_confidence,
                config.score_threshold, config.nms_threshold, adapt_scaling, roi_offset);
    }

    auto generate_openvino_request(const Image& image) noexcept
        -> std::expected<ov::InferRequest, std::string> {

        const auto& origin_mat = image.details().mat;
        if (origin_mat.empty()) [[unlikely]] {
            return std::unexpected { "Empty image mat" };
        }

        auto segmentation = origin_mat;
        roi_offset        = { 0.f, 0.f };
        if (config.use_roi_segment) {
            auto action_success = false;
            do {
                const auto cols = config.roi_cols;
                const auto rows = config.roi_rows;
                if (cols > origin_mat.cols) break;
                if (rows > origin_mat.rows) break;

                const auto x    = (origin_mat.cols - cols) / 2;
                const auto y    = (origin_mat.rows - rows) / 2;
                const auto rect = cv::Rect2i { x, y, cols, rows };

                action_success = true;
                roi_offset     = {
                    static_cast<float>(x),
                    static_cast<float>(y),
                };
                segmentation = origin_mat(rect);
            } while (false);

            if (!action_success) {
                return std::unexpected { "Failed to segment image" };
            }
        }

        const auto rows   = static_cast<int>(input_dimensions.H);
        const auto cols   = static_cast<int>(input_dimensions.W);
        auto input_tensor = ov::Tensor {
            ov::element::u8,
            input_layout.shape(input_dimensions),
        };
        {
            adapt_scaling = std::min(static_cast<float>(1. * cols / segmentation.cols),
                static_cast<float>(1. * rows / segmentation.rows));

            const auto scaled_w = static_cast<int>(1. * segmentation.cols * adapt_scaling);
            const auto scaled_h = static_cast<int>(1. * segmentation.rows * adapt_scaling);

            auto input_mat = cv::Mat { rows, cols, CV_8UC3, input_tensor.data() };
            input_mat.setTo(cv::Scalar::all(0));

            auto input_roi = cv::Rect2i { 0, 0, scaled_w, scaled_h };
            cv::resize(segmentation, input_mat(input_roi), { scaled_w, scaled_h });
        }

        auto request = openvino_model.create_infer_request();
        request.set_input_tensor(input_tensor);

        return request;
    }

    template <class raw_type>
    static auto cast_to_armor_result(raw_type raw) noexcept -> Armor2D {
        auto armor = Armor2D {};

        armor.genre = raw.armor_genre();
        armor.color = raw.armor_color();
        armor.shape =
            DeviceIds::kLargeArmor().contains(armor.genre) ? ArmorShape::LARGE : ArmorShape::SMALL;

        armor.tl = raw.tl();
        armor.tr = raw.tr();
        armor.br = raw.br();
        armor.bl = raw.bl();

        armor.confidence = raw.confidence();
        armor.center     = (armor.tl + armor.tr + armor.br + armor.bl) * 0.25f;

        return armor;
    }

    auto explain_infer_result(ov::InferRequest& finished_request) const noexcept
        -> std::optional<Armor2Ds> {
        if (!explain_infer_functor) {
            return std::nullopt;
        }
        return explain_infer_functor->explain(finished_request);
    }

    auto sync_detect(const Image& image) noexcept -> std::optional<Armor2Ds> {
        auto result = generate_openvino_request(image);
        if (!result.has_value()) {
            return std::nullopt;
        }

        auto request = std::move(result.value());
        request.infer();

        return explain_infer_result(request);
    }
};

auto ArmorDetection::initialize(const YAML::Node& yaml) noexcept
    -> std::expected<void, std::string> {
    return pimpl->configure(yaml);
}

auto ArmorDetection::sync_detect(const Image& image) noexcept
    -> std::optional<std::vector<Armor2D>> {
    return pimpl->sync_detect(image);
}

ArmorDetection::ArmorDetection() noexcept
    : pimpl { std::make_unique<Impl>() } { }

ArmorDetection::~ArmorDetection() noexcept = default;
