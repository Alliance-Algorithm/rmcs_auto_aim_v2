#pragma once

#include <eigen3/Eigen/Dense>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/mat.hpp>

template <std::size_t cols, typename scale>
static auto cast_opencv_matrix(std::array<scale, cols>& source) {
    auto mat_type = int {};
    /*  */ if constexpr (std::same_as<scale, double>) {
        mat_type = CV_64FC1;
    } else if constexpr (std::same_as<scale, float>) {
        mat_type = CV_32FC1;
    } else if constexpr (std::same_as<scale, int>) {
        mat_type = CV_32SC1;
    } else {
        static_assert(false, "Unsupport mat scale type");
    }

    return cv::Mat { 1, cols, mat_type, source.data() };
}

template <std::size_t cols, std::size_t rows, typename scale>
static auto cast_opencv_matrix(std::array<std::array<scale, cols>, rows>& source) {
    auto mat_type = int {};
    /*  */ if constexpr (std::same_as<scale, double>) {
        mat_type = CV_64FC1;
    } else if constexpr (std::same_as<scale, float>) {
        mat_type = CV_32FC1;
    } else if constexpr (std::same_as<scale, int>) {
        mat_type = CV_32SC1;
    } else {
        static_assert(false, "Unsupport mat scale type");
    }

    return cv::Mat { rows, cols, mat_type, source[0].data() };
}

template <typename Input, typename Output>
concept ConvertibleTo = std::is_convertible_v<Input, Output>;
/*  @Note: Row-Major */
template <typename input_type, std::size_t N, typename output_type, std::size_t rows,
    std::size_t cols>
    requires ConvertibleTo<input_type, output_type>
static auto reshape_array(std::array<input_type, N> const& input_array)
    -> std::array<std::array<output_type, cols>, rows> {
    static_assert(N == rows * cols, "input_array的元素总数N必须等于rows*cols");
    using ResultArray = std::array<std::array<output_type, cols>, rows>;

    ResultArray result_array;
    for (std::size_t i = 0; i < N; ++i) {
        std::size_t row_idx = i / cols;
        std::size_t col_idx = i % cols;

        result_array[row_idx][col_idx] = input_array[i];
    }
    return result_array;
}

/*  @Note: Row-Major */
template <typename input_type, std::size_t N, typename output_type>
    requires ConvertibleTo<input_type, output_type>
static auto reshape_array(std::array<input_type, N> const& input_array)
    -> std::array<output_type, N> {
    using ResultArray = std::array<output_type, N>;

    ResultArray result_array;
    std::transform(input_array.begin(), input_array.end(), result_array.begin(),
        [](const input_type& val) { return static_cast<output_type>(val); });

    return result_array;
}
