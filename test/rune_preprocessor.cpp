#include "module/identifier/rune/rune_preprocessor.hpp"

#include <memory>
#include <string>
#include <string_view>

#include <gtest/gtest.h>
#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>

#include "utility/image/image.details.hpp"

using namespace rmcs;

namespace {

// 测试辅助函数：用于构造最小配置和 BGR 测试图像
auto make_config(std::string_view target_color = "red", int binary_threshold = 40,
    double min_contour_area = 20.0, double max_contour_area = 10000.0) -> YAML::Node {
    auto config                    = YAML::Node {};
    config["target_color"]        = std::string { target_color };
    config["binary_threshold"]    = binary_threshold;
    config["min_contour_area"]    = min_contour_area;
    config["max_contour_area"]    = max_contour_area;
    return config;
}

auto make_bgr_image(int rows, int cols, const cv::Scalar& color = cv::Scalar::all(0))
    -> std::unique_ptr<Image> {
    auto image           = std::make_unique<Image>();
    image->details().mat = cv::Mat(rows, cols, CV_8UC3, color).clone();
    return image;
}

TEST(RunePreprocessor, InitializeAcceptsValidConfig) {
    auto preprocessor = identifier::RunePreprocessor {};
    auto result       = preprocessor.initialize(make_config());

    ASSERT_TRUE(result.has_value()) << result.error();
}

// 测试初始化阶段对配置项合法性的校验
TEST(RunePreprocessor, InitializeRejectsUnknownTargetColor) {
    auto preprocessor = identifier::RunePreprocessor {};
    auto result       = preprocessor.initialize(make_config("green"));

    ASSERT_FALSE(result.has_value());
    EXPECT_NE(result.error().find("target_color"), std::string::npos);
}

TEST(RunePreprocessor, InitializeRejectsInvertedAreaRange) {
    auto preprocessor = identifier::RunePreprocessor {};
    auto result       = preprocessor.initialize(make_config("red", 40, 100.0, 10.0));

    ASSERT_FALSE(result.has_value());
    EXPECT_NE(result.error().find("max_contour_area"), std::string::npos);
}

TEST(RunePreprocessor, InitializeRejectsIdentifierParentNode) {
    auto preprocessor = identifier::RunePreprocessor {};

    auto config                        = YAML::Node {};
    config["identifier"]["rune_preprocessor"] = make_config();

    auto result = preprocessor.initialize(config["identifier"]);

    ASSERT_FALSE(result.has_value());
    EXPECT_NE(result.error().find("target_color"), std::string::npos);
}

// 测试 sync_process 在运行前对初始化状态、空图像和图像类型的校验
TEST(RunePreprocessor, SyncProcessRejectsUninitializedTargetColor) {
    auto preprocessor = identifier::RunePreprocessor {};
    auto image        = make_bgr_image(8, 8);

    auto result = preprocessor.sync_process(*image);

    ASSERT_FALSE(result.has_value());
}

TEST(RunePreprocessor, SyncProcessRejectsEmptyImage) {
    auto preprocessor = identifier::RunePreprocessor {};
    ASSERT_TRUE(preprocessor.initialize(make_config()).has_value());

    auto empty_image = Image {};
    auto result      = preprocessor.sync_process(empty_image);

    ASSERT_FALSE(result.has_value());
}

TEST(RunePreprocessor, SyncProcessRejectsNonBgrImageType) {
    auto preprocessor = identifier::RunePreprocessor {};
    ASSERT_TRUE(preprocessor.initialize(make_config()).has_value());

    auto image          = Image {};
    image.details().mat = cv::Mat(10, 10, CV_8UC1, cv::Scalar::all(0));

    auto result = preprocessor.sync_process(image);

    ASSERT_FALSE(result.has_value());
}

// 测试根据目标颜色选择红减蓝或蓝减红的通道差分逻辑
TEST(RunePreprocessor, SyncProcessUsesRedMinusBlueForRedTargets) {
    auto preprocessor = identifier::RunePreprocessor {};
    ASSERT_TRUE(preprocessor.initialize(make_config("red", 1, 10.0, 1000.0)).has_value());

    auto image = make_bgr_image(16, 16);
    cv::rectangle(image->details().mat, cv::Rect(0, 0, 8, 16), cv::Scalar(10, 0, 100), cv::FILLED);
    cv::rectangle(image->details().mat, cv::Rect(8, 0, 8, 16), cv::Scalar(100, 0, 10), cv::FILLED);

    auto result = preprocessor.sync_process(*image);

    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result->candidates.size(), 1u);
    EXPECT_EQ(result->candidates.front().bounding_rect.x, 0);
}

TEST(RunePreprocessor, SyncProcessUsesBlueMinusRedForBlueTargets) {
    auto preprocessor = identifier::RunePreprocessor {};
    ASSERT_TRUE(preprocessor.initialize(make_config("blue", 1, 10.0, 1000.0)).has_value());

    auto image = make_bgr_image(16, 16);
    cv::rectangle(image->details().mat, cv::Rect(0, 0, 8, 16), cv::Scalar(10, 0, 100), cv::FILLED);
    cv::rectangle(image->details().mat, cv::Rect(8, 0, 8, 16), cv::Scalar(100, 0, 10), cv::FILLED);

    auto result = preprocessor.sync_process(*image);

    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result->candidates.size(), 1u);
    EXPECT_EQ(result->candidates.front().bounding_rect.x, 8);
}

// 测试按面积筛选轮廓，并将保留轮廓构造成候选结果
TEST(RunePreprocessor, SyncProcessFiltersContoursByAreaAndBuildsCandidates) {
    auto preprocessor = identifier::RunePreprocessor {};
    ASSERT_TRUE(preprocessor.initialize(make_config("red", 1, 50.0, 1000.0)).has_value());

    auto image = make_bgr_image(80, 80);
    cv::rectangle(image->details().mat, cv::Rect(5, 5, 3, 3), cv::Scalar(0, 0, 255), cv::FILLED);
    cv::rectangle(
        image->details().mat, cv::Rect(20, 20, 20, 15), cv::Scalar(0, 0, 255), cv::FILLED);

    auto result = preprocessor.sync_process(*image);

    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result->candidates.size(), 1u);
    ASSERT_EQ(result->hierarchy.size(), 1u);

    const auto& candidate = result->candidates.front();
    EXPECT_GT(candidate.area, 50.0);
    EXPECT_EQ(candidate.bounding_rect.x, 20);
    EXPECT_EQ(candidate.bounding_rect.y, 20);
    EXPECT_EQ(candidate.bounding_rect.width, 20);
    EXPECT_EQ(candidate.bounding_rect.height, 15);
    EXPECT_NEAR(candidate.center.x, 29.5f, 1.0f);
    EXPECT_NEAR(candidate.center.y, 27.0f, 1.0f);
    EXPECT_FALSE(candidate.contour.empty());
}

// 测试轮廓过滤后层级关系的保留、重连以及子轮廓提升行为
TEST(RunePreprocessor, SyncProcessPreservesFilteredHierarchyRelationships) {
    auto preprocessor = identifier::RunePreprocessor {};
    ASSERT_TRUE(preprocessor.initialize(make_config("red", 1, 20.0, 10000.0)).has_value());

    auto image = make_bgr_image(120, 120);
    cv::rectangle(
        image->details().mat, cv::Rect(10, 10, 80, 80), cv::Scalar(0, 0, 255), cv::FILLED);
    cv::rectangle(image->details().mat, cv::Rect(30, 30, 40, 40), cv::Scalar::all(0), cv::FILLED);

    auto result = preprocessor.sync_process(*image);

    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result->candidates.size(), 2u);
    ASSERT_EQ(result->hierarchy.size(), 2u);

    const auto root_index  = result->hierarchy[0][3] == -1 ? 0u : 1u;
    const auto child_index = root_index == 0u ? 1u : 0u;

    EXPECT_EQ(result->hierarchy[root_index][2], static_cast<int>(child_index));
    EXPECT_EQ(result->hierarchy[child_index][3], static_cast<int>(root_index));
}

TEST(RunePreprocessor, SyncProcessRemovesFilteredContoursFromHierarchyChain) {
    auto preprocessor = identifier::RunePreprocessor {};
    ASSERT_TRUE(preprocessor.initialize(make_config("red", 1, 200.0, 10000.0)).has_value());

    auto image = make_bgr_image(120, 160);
    cv::rectangle(
        image->details().mat, cv::Rect(10, 10, 120, 80), cv::Scalar(0, 0, 255), cv::FILLED);
    cv::rectangle(image->details().mat, cv::Rect(20, 20, 10, 10), cv::Scalar::all(0), cv::FILLED);
    cv::rectangle(image->details().mat, cv::Rect(70, 20, 30, 30), cv::Scalar::all(0), cv::FILLED);

    auto result = preprocessor.sync_process(*image);

    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result->candidates.size(), 2u);
    ASSERT_EQ(result->hierarchy.size(), 2u);

    const auto root_index  = result->hierarchy[0][3] == -1 ? 0u : 1u;
    const auto child_index = root_index == 0u ? 1u : 0u;

    EXPECT_EQ(result->hierarchy[root_index][2], static_cast<int>(child_index));
    EXPECT_EQ(result->hierarchy[child_index][3], static_cast<int>(root_index));
    EXPECT_EQ(result->hierarchy[child_index][0], -1);
    EXPECT_EQ(result->hierarchy[child_index][1], -1);
}

TEST(RunePreprocessor, SyncProcessPromotesKeptChildWhenFilteredParentRemoved) {
    auto preprocessor = identifier::RunePreprocessor {};
    ASSERT_TRUE(preprocessor.initialize(make_config("red", 1, 20.0, 1500.0)).has_value());

    auto image = make_bgr_image(140, 180);
    cv::rectangle(
        image->details().mat, cv::Rect(10, 10, 80, 80), cv::Scalar(0, 0, 255), cv::FILLED);
    cv::rectangle(image->details().mat, cv::Rect(30, 30, 20, 20), cv::Scalar::all(0), cv::FILLED);
    cv::rectangle(
        image->details().mat, cv::Rect(110, 10, 20, 20), cv::Scalar(0, 0, 255), cv::FILLED);

    auto result = preprocessor.sync_process(*image);

    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result->candidates.size(), 2u);
    ASSERT_EQ(result->hierarchy.size(), 2u);

    auto root_count           = 0u;
    auto promoted_child_index = std::size_t { 0 };
    for (std::size_t index = 0; index < result->hierarchy.size(); ++index) {
        if (result->hierarchy[index][3] == -1) {
            ++root_count;
        }
        if (result->candidates[index].bounding_rect.x == 30
            && result->candidates[index].bounding_rect.y == 30) {
            promoted_child_index = index;
        }
    }

    EXPECT_EQ(root_count, 2u);
    EXPECT_EQ(result->hierarchy[promoted_child_index][3], -1);

    const auto sibling_index = result->hierarchy[promoted_child_index][0] != -1
        ? static_cast<std::size_t>(result->hierarchy[promoted_child_index][0])
        : static_cast<std::size_t>(result->hierarchy[promoted_child_index][1]);
    EXPECT_NE(sibling_index, promoted_child_index);
    EXPECT_EQ(result->hierarchy[sibling_index][3], -1);
}

} // namespace
