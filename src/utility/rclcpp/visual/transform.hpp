#pragma once

#include "utility/math/linear.hpp"
#include "utility/pimpl.hpp"
#include "utility/rclcpp/node.hpp"

namespace rmcs::util::visual {

using rmcs::Transform;

struct DynamicTransform {
    RMCS_PIMPL_DEFINITION(DynamicTransform)

public:
    explicit DynamicTransform(RclcppNode&);

    auto set_link(const std::string& parent, const std::string& child) -> void;

    auto publish(const Transform&) -> void;
};

}
