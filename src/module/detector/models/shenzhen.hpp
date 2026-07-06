#pragma once
#include "utility/model/armor_detection.hpp"
#include "utility/model/common_model.hpp"

#include <openvino/core/preprocess/pre_post_process.hpp>
#include <openvino/runtime/compiled_model.hpp>

namespace rmcs {

struct ShenZhenBasic {
    TensorLayout input_layout = TensorLayout::from<"NHWC">();
    TensorLayout model_layout = TensorLayout::from<"NCHW">();
    Dimensions dimensions     = Dimensions { .W = 640, .H = 640 };
    std::string infer_device  = "AUTO";

    auto compile(ov::Core& core, std::string_view location) const -> ov::CompiledModel {
        auto raw = core.read_model(std::string { location });
        auto ppp = ov::preprocess::PrePostProcessor { raw };
        {
            auto& input = ppp.input();
            input.tensor()
                .set_element_type(ov::element::u8)
                .set_shape(input_layout.partial_shape(dimensions))
                .set_layout(input_layout.layout())
                .set_color_format(ov::preprocess::ColorFormat::BGR);
            input.preprocess()
                .convert_element_type(ov::element::f32)
                .convert_color(ov::preprocess::ColorFormat::RGB)
                .scale({ 255., 255., 255. });
            input.model().set_layout(model_layout.layout());
        }
        {
            auto& output = ppp.output();
            output.tensor().set_element_type(ov::element::f32);
        }
        return core.compile_model(ppp.build(), infer_device, kRealTimePerformanceMode);
    }

    struct ResultData {
        using precision_type = float;

        struct Corners {
            precision_type lt_x;
            precision_type lt_y;
            precision_type lb_x;
            precision_type lb_y;
            precision_type rb_x;
            precision_type rb_y;
            precision_type rt_x;
            precision_type rt_y;
        } corners;

        precision_type confidence;

        struct Color {
            precision_type blue;
            precision_type red;
            precision_type dark;
            precision_type mix;
        } color;

        struct Genre {
            precision_type sentry;
            precision_type hero;
            precision_type engineer;
            precision_type infantry_3;
            precision_type infantry_4;
            precision_type infantry_5;
            precision_type outpost;
            precision_type base_small;
            precision_type base_large;
        } genre;
    };

    using Result = InferResultAdapter<ResultData>;
};

}
