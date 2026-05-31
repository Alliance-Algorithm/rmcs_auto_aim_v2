#include <filesystem>
#include <print>
#include <sstream>

#include <openvino/core/layout.hpp>
#include <openvino/runtime/core.hpp>

auto main(int argc, char* argv[]) -> int {
    std::println("Argc: {}", argc);
    if (argc != 2) {
        std::println("Only model location can be passed to this tool");
        return EXIT_FAILURE;
    }

    const auto filename = std::filesystem::path { argv[1] };
    std::println("Filename: {}", filename.string());
    if (!std::filesystem::exists(filename)) {
        std::println("The model file does not exist in your filesystem");
        return EXIT_FAILURE;
    }

    auto as_text = [](const auto& value) {
        auto stream = std::ostringstream {};
        stream << value;
        return stream.str();
    };

    try {
        const auto ov_core  = ov::Core {};
        const auto ov_model = ov_core.read_model(filename);

        std::println("==== Model Basic Status ====");
        std::println("Model loaded: {}", filename.string());
        std::println("Input count : {}", ov_model->inputs().size());
        std::println("Output count: {}", ov_model->outputs().size());

        std::println("---- Inputs ----");
        for (const auto& input : ov_model->inputs()) {
            const auto shape  = input.get_partial_shape();
            const auto layout = ov::layout::get_layout(input);

            std::println("Name        : {}", input.get_any_name());
            std::println("Element type: {}", as_text(input.get_element_type()));
            std::println("Shape       : {}", as_text(shape));
            std::println("Static shape: {}", shape.is_static() ? "yes" : "no");
            std::println("Layout      : {}", layout.empty() ? "<not set>" : layout.to_string());
            std::println("Color format: <not stored in model metadata>");
            std::println();
        }

        std::println("---- Outputs ----");
        for (const auto& output : ov_model->outputs()) {
            std::println("Name        : {}", output.get_any_name());
            std::println("Element type: {}", as_text(output.get_element_type()));
            std::println("Shape       : {}", as_text(output.get_partial_shape()));
            std::println();
        }
    } catch (const std::exception& e) {
        std::println("Failed to read model: {}", e.what());
        return EXIT_FAILURE;
    }

    return 0;
}
