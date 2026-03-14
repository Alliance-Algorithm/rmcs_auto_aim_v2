#include "armor_detection.hpp"

#include "module/identifier/models/shenzhen_0526.hpp"
#include "module/identifier/models/shenzhen_0708.hpp"
#include "module/identifier/models/tongji_yolov5.hpp"

#include "utility/image/image.details.hpp"
#include "utility/math/sigmoid.hpp"
#include "utility/model/common_model.hpp"
#include "utility/robot/id.hpp"
#include "utility/serializable.hpp"

#include <algorithm>
#include <concepts>
#include <filesystem>
#include <format>
#include <memory>
#include <span>

#include <opencv2/dnn/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <openvino/runtime/compiled_model.hpp>
#include <openvino/runtime/core.hpp>

namespace {

template <class>
inline constexpr bool kUnsupportedPrecision = false;

template <class precision_type>
auto validate_output_tensor_precision(const ov::Tensor& tensor) noexcept
    -> std::expected<void, std::string> {
    if constexpr (std::same_as<precision_type, float>) {
        if (tensor.get_element_type() != ov::element::f32) {
            return std::unexpected { "Unexpected output tensor element type: expected f32" };
        }
        return {};
    } else {
        static_assert(
            kUnsupportedPrecision<precision_type>, "Unsupported output tensor precision type");
    }
}

}

using namespace rmcs::identifier;

struct ArmorDetection::Impl {
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

    struct Segmentation {
        cv::Mat mat {};
        cv::Point2f roi_offset { 0.f, 0.f };
    };

    struct Projection {
        cv::Point2f roi_offset { 0.f, 0.f };
        cv::Point2f input_offset { 0.f, 0.f };
        cv::Point2f input_scale { 1.f, 1.f };

        auto restore(cv::Point2f point) const noexcept -> cv::Point2f {
            return {
                (point.x - input_offset.x) / input_scale.x + roi_offset.x,
                (point.y - input_offset.y) / input_scale.y + roi_offset.y,
            };
        }
    };

    struct InferContext {
        ov::InferRequest request;
        Projection projection {};
    };

    struct PipelineInterface {
        virtual ~PipelineInterface() = default;

        virtual auto generate_request(ov::CompiledModel&, const Image&) noexcept
            -> std::expected<InferContext, std::string> = 0;
        virtual auto explain(InferContext&) const noexcept
            -> std::expected<Armor2Ds, std::string> = 0;
    };

    template <class model_type>
    struct PipelineBase : PipelineInterface {
        using result_type = typename model_type::Result;

        TensorLayout input_layout;
        Dimensions input_dimensions;
        float min_confidence;
        float score_threshold;
        float nms_threshold;
        Impl& owner;

        explicit PipelineBase(Impl& owner, const model_type& model, float min_confidence,
            float score_threshold, float nms_threshold) noexcept
            : input_layout { model.input_layout }
            , input_dimensions { model.dimensions }
            , min_confidence { min_confidence }
            , score_threshold { score_threshold }
            , nms_threshold { nms_threshold }
            , owner { owner } { }

        auto explain(InferContext& finished_context) const noexcept
            -> std::expected<Armor2Ds, std::string> override {
            return Impl::explain_result<result_type>(finished_context.request,
                finished_context.projection, min_confidence, score_threshold, nms_threshold);
        }

    protected:
        auto make_input_tensor() const noexcept -> ov::Tensor {
            return {
                ov::element::u8,
                input_layout.shape(input_dimensions),
            };
        }
    };

    template <class model_type>
    struct TongJiPipeline final : PipelineBase<model_type> {
        using Base = PipelineBase<model_type>;

        explicit TongJiPipeline(Impl& owner, const model_type& model, float min_confidence,
            float score_threshold, float nms_threshold) noexcept
            : Base { owner, model, min_confidence, score_threshold, nms_threshold } { }

        auto generate_request(ov::CompiledModel& compiled_model, const Image& image) noexcept
            -> std::expected<InferContext, std::string> override {

            auto segmented = this->owner.segment_image(image);
            if (!segmented.has_value()) {
                return std::unexpected { segmented.error() };
            }

            const auto rows   = static_cast<int>(this->input_dimensions.H);
            const auto cols   = static_cast<int>(this->input_dimensions.W);
            auto input_tensor = this->make_input_tensor();
            auto& source      = segmented->mat;

            const auto scaling = std::min(
                static_cast<float>(cols) / source.cols, static_cast<float>(rows) / source.rows);
            const auto scaled_w = static_cast<int>(1.0 * source.cols * scaling);
            const auto scaled_h = static_cast<int>(1.0 * source.rows * scaling);

            auto projection = Projection {
                .roi_offset  = segmented->roi_offset,
                .input_scale = { scaling, scaling },
            };
            auto input_mat = cv::Mat { rows, cols, CV_8UC3, input_tensor.data() };
            input_mat.setTo(cv::Scalar::all(0));
            cv::resize(
                source, input_mat(cv::Rect2i { 0, 0, scaled_w, scaled_h }), { scaled_w, scaled_h });

            auto request = compiled_model.create_infer_request();
            request.set_input_tensor(input_tensor);
            return InferContext {
                .request    = std::move(request),
                .projection = projection,
            };
        }
    };

    template <class model_type>
    struct ShenZhenPipeline final : PipelineBase<model_type> {
        using Base = PipelineBase<model_type>;

        explicit ShenZhenPipeline(Impl& owner, const model_type& model, float min_confidence,
            float score_threshold, float nms_threshold) noexcept
            : Base { owner, model, min_confidence, score_threshold, nms_threshold } { }

        auto generate_request(ov::CompiledModel& compiled_model, const Image& image) noexcept
            -> std::expected<InferContext, std::string> override {

            auto segmented = this->owner.segment_image(image);
            if (!segmented.has_value()) {
                return std::unexpected { segmented.error() };
            }

            const auto rows   = static_cast<int>(this->input_dimensions.H);
            const auto cols   = static_cast<int>(this->input_dimensions.W);
            auto input_tensor = this->make_input_tensor();
            auto& source      = segmented->mat;

            auto projection = Projection {
                .roi_offset  = segmented->roi_offset,
                .input_scale = {
                    static_cast<float>(cols) / source.cols,
                    static_cast<float>(rows) / source.rows,
                },
            };
            auto input_mat = cv::Mat { rows, cols, CV_8UC3, input_tensor.data() };
            cv::resize(source, input_mat, input_mat.size());

            auto request = compiled_model.create_infer_request();
            request.set_input_tensor(input_tensor);
            return InferContext {
                .request    = std::move(request),
                .projection = projection,
            };
        }
    };

    ov::Core openvino_core;
    ov::CompiledModel openvino_model;
    std::unique_ptr<PipelineInterface> infer_pipeline;

    static auto supported_models_message() -> std::string {
        return std::format("tongji_yolov5 ({}), shenzhen_0526 ({}), shenzhen_0708 ({})",
            TongJiYoloV5::kLocation, ShenZhen0526::kLocation, ShenZhen0708::kLocation);
    }

    auto configure(const YAML::Node& yaml) noexcept -> std::expected<void, std::string> {
        auto result = config.serialize(yaml);
        if (!result.has_value()) {
            return std::unexpected { result.error() };
        }

        const auto model_name = std::filesystem::path { config.model_location }.filename().string();

        if (model_name == TongJiYoloV5::kLocation) {
            compile_model_with<TongJiYoloV5, TongJiPipeline>();
            return {};
        }
        if (model_name == ShenZhen0526::kLocation) {
            compile_model_with<ShenZhen0526, ShenZhenPipeline>();
            return {};
        }
        if (model_name == ShenZhen0708::kLocation) {
            compile_model_with<ShenZhen0708, ShenZhenPipeline>();
            return {};
        }

        return std::unexpected { std::format("Unsupported model type: {}. Supported models: {}",
            model_name, supported_models_message()) };
    }

    template <class model_type, template <class> class pipeline_type>
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

        openvino_model = model.compile(openvino_core, config.model_location);
        infer_pipeline = std::make_unique<pipeline_type<model_type>>(
            *this, model, config.min_confidence, config.score_threshold, config.nms_threshold);
    }

    auto segment_image(const Image& image) const noexcept
        -> std::expected<Segmentation, std::string> {
        const auto& origin_mat = image.details().mat;
        if (origin_mat.empty()) [[unlikely]] {
            return std::unexpected { "Empty image mat" };
        }

        auto segmented = Segmentation {
            .mat = origin_mat,
        };
        if (!config.use_roi_segment) {
            return segmented;
        }

        const auto cols = config.roi_cols;
        const auto rows = config.roi_rows;
        if (cols <= 0 || rows <= 0) {
            return std::unexpected { std::format("Invalid ROI size: cols={} rows={}", cols, rows) };
        }
        if (cols > origin_mat.cols || rows > origin_mat.rows) {
            return std::unexpected { std::format("ROI exceeds image size: roi={}x{}, image={}x{}",
                cols, rows, origin_mat.cols, origin_mat.rows) };
        }

        const auto x    = (origin_mat.cols - cols) / 2;
        const auto y    = (origin_mat.rows - rows) / 2;
        const auto rect = cv::Rect2i { x, y, cols, rows };

        segmented.roi_offset = {
            static_cast<float>(x),
            static_cast<float>(y),
        };
        segmented.mat = origin_mat(rect);
        return segmented;
    }

    template <class candidate_type>
    static auto cast_to_armor_result(
        candidate_type armor_candidate, const Projection& projection) noexcept -> Armor2D {
        auto armor = Armor2D {};

        armor.genre = armor_candidate.armor_genre();
        armor.color = armor_candidate.armor_color();
        armor.shape =
            DeviceIds::kLargeArmor().contains(armor.genre) ? ArmorShape::LARGE : ArmorShape::SMALL;

        armor.tl = projection.restore(armor_candidate.tl());
        armor.tr = projection.restore(armor_candidate.tr());
        armor.br = projection.restore(armor_candidate.br());
        armor.bl = projection.restore(armor_candidate.bl());

        armor.confidence = armor_candidate.confidence();
        armor.center     = (armor.tl + armor.tr + armor.br + armor.bl) * 0.25f;

        return armor;
    }

    template <class result_type>
    static auto explain_result(ov::InferRequest& finished_request, const Projection& projection,
        float min_confidence, float score_threshold, float nms_threshold) noexcept
        -> std::expected<Armor2Ds, std::string> {
        using precision_type = typename result_type::precision_type;

        auto tensor = finished_request.get_output_tensor();
        if (auto precision_check = validate_output_tensor_precision<precision_type>(tensor);
            !precision_check.has_value()) {
            return std::unexpected { precision_check.error() };
        }
        auto& shape = tensor.get_shape();

        if (shape.size() != 3) {
            return std::unexpected {
                std::format("Unexpected output tensor rank: {}", shape.size()),
            };
        }

        const auto rows = static_cast<std::size_t>(shape.at(1));
        const auto cols = static_cast<std::size_t>(shape.at(2));
        if (cols != result_type::length()) {
            return std::unexpected { std::format("Unexpected output tensor width: expected {}, got "
                                                 "{}",
                result_type::length(), cols) };
        }

        auto parsed_results = std::vector<result_type> {};
        auto scores         = std::vector<float> {};
        auto boxes          = std::vector<cv::Rect> {};

        const auto* data = tensor.data<precision_type>();
        for (std::size_t row = 0; row < rows; row++) {
            auto armor_candidate = result_type {};
            armor_candidate.unsafe_from(std::span { data + row * cols, cols });
            armor_candidate.confidence() = util::sigmoid(armor_candidate.confidence());

            if (armor_candidate.confidence() > min_confidence) {
                parsed_results.push_back(armor_candidate);
                scores.push_back(static_cast<float>(armor_candidate.confidence()));
                boxes.push_back(static_cast<cv::Rect>(armor_candidate.bounding_rect()));
            }
        }

        auto kept_points = std::vector<int> {};
        cv::dnn::NMSBoxes(boxes, scores, score_threshold, nms_threshold, kept_points);

        auto armors = Armor2Ds {};
        armors.reserve(kept_points.size());

        for (auto idx : kept_points) {
            armors.push_back(cast_to_armor_result(parsed_results[idx], projection));
        }
        return armors;
    }

    auto generate_openvino_request(const Image& image) noexcept
        -> std::expected<InferContext, std::string> {
        if (!infer_pipeline) [[unlikely]] {
            return std::unexpected { "Infer pipeline has not been initialized" };
        }
        return infer_pipeline->generate_request(openvino_model, image);
    }

    auto explain_infer_result(InferContext& finished_context) const noexcept
        -> std::expected<Armor2Ds, std::string> {
        if (!infer_pipeline) {
            return std::unexpected { "Infer pipeline has not been initialized" };
        }
        return infer_pipeline->explain(finished_context);
    }

    auto sync_detect(const Image& image) noexcept -> std::expected<Armor2Ds, std::string> {
        auto result = generate_openvino_request(image);
        if (!result.has_value()) {
            return std::unexpected { result.error() };
        }

        auto context = std::move(result.value());
        context.request.infer();

        return explain_infer_result(context);
    }
};

auto ArmorDetection::initialize(const YAML::Node& yaml) noexcept
    -> std::expected<void, std::string> {
    return pimpl->configure(yaml);
}

auto ArmorDetection::sync_detect(const Image& image) noexcept
    -> std::expected<std::vector<Armor2D>, std::string> {
    return pimpl->sync_detect(image);
}

ArmorDetection::ArmorDetection() noexcept
    : pimpl { std::make_unique<Impl>() } { }

ArmorDetection::~ArmorDetection() noexcept = default;
