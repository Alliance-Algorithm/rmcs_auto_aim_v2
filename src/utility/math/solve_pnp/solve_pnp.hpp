#pragma once

#include <eigen3/Eigen/Dense>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/mat.hpp>

template <std::size_t cols, typename scale>
/**
 * @brief 将一维 std::array 视为单行 cv::Mat（不复制底层数据）。
 *
 * 构造并返回一个 1 x cols 的 cv::Mat，矩阵类型由元素类型 `scale` 决定：
 * - `double` -> `CV_64FC1`
 * - `float`  -> `CV_32FC1`
 * - `int`    -> `CV_32SC1`
 * 返回的 cv::Mat 使用 source.data() 作为底层数据指针（不进行数据拷贝）。
 *
 * @tparam cols 数组列数（矩阵的列数）。
 * @tparam scale 数组元素类型，用于决定返回矩阵的 OpenCV 类型。
 * @param source 用于作为矩阵数据的源数组；返回的 cv::Mat 与该数组共享内存。
 * @return cv::Mat 大小为 1 x cols，类型如上所述，数据指向 source.data()。
 */
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
/**
 * 将二维 std::array 视作具有指定行列和元素类型的 cv::Mat，且不复制底层数据。
 *
 * 仅支持元素类型 `scale` 为 `double`、`float` 或 `int`；其它类型将在编译时报错。
 *
 * @tparam cols 列数（矩阵的列数）。
 * @tparam rows 行数（矩阵的行数）。
 * @tparam scale 元素类型，决定返回矩阵的像素深度（仅限 double/float/int）。
 * @param source 用于作为矩阵数据的二维 std::array，按行主序组织。
 * @return cv::Mat 大小为 rows x cols，像素深度对应 `scale`，矩阵的数据指向传入数组的内存（不进行数据复制）。
 */
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
/**
 * @brief 将一维平铺数组按行主序重塑为指定尺寸的二维 std::array。
 *
 * @tparam input_type 源数组元素类型。
 * @tparam N 源数组元素数量，必须等于 rows * cols。
 * @tparam output_type 目标二维数组元素类型，可由 input_type 转换得到。
 * @tparam rows 结果二维数组的行数。
 * @tparam cols 结果二维数组的列数。
 * @param input_array 包含 N 个元素的输入一维数组，按行主序填充。
 * @return std::array<std::array<output_type, cols>, rows> 按行主序重排后的 rows x cols 二维数组。
 *
 * @note 编译时断言要求 N == rows * cols。
 */
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
/**
 * @brief 将一维 std::array 的元素逐元素转换为指定的输出类型并返回新的数组。
 *
 * @tparam input_type 源数组元素类型。
 * @tparam N 数组长度。
 * @tparam output_type 目标元素类型。
 * @param input_array 要转换的源数组，其每个元素会使用 `static_cast<output_type>` 进行转换。
 * @return std::array<output_type, N> 包含转换后元素的新数组。
 */
static auto reshape_array(std::array<input_type, N> const& input_array)
    -> std::array<output_type, N> {
    using ResultArray = std::array<output_type, N>;

    ResultArray result_array;
    std::transform(input_array.begin(), input_array.end(), result_array.begin(),
        [](const input_type& val) { return static_cast<output_type>(val); });

    return result_array;
}