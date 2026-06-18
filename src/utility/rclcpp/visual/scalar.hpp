#pragma once

#include "utility/pimpl.hpp"
#include "utility/rclcpp/node.hpp"

namespace rmcs::util::visual {

struct Scalar {
    RMCS_PIMPL_DEFINITION(Scalar)

public:
    Scalar(RclcppNode&, std::string name);

    Scalar(Scalar&&) noexcept;
    Scalar& operator=(Scalar&&) noexcept;

    auto publish(double value) -> void;
};

}
