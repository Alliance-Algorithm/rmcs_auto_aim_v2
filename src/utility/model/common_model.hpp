#pragma once
#include <openvino/runtime/compiled_model.hpp>
#include <string>

namespace rmcs {

struct OvModel {
    ov::CompiledModel compiled_model;

    explicit OvModel(const std::string& location) { }
};

}
