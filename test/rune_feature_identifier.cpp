#include "module/identifier/rune/rune_feature_identifier.hpp"
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

auto make_preprocessor_config(std::string_view target_color = "red", int binary_threshold = 40,
    double min_contour_area = 20.0, double max_contour_area = 10000.0) -> YAML::Node {
    auto config                = YAML::Node {};
    config["target_color"]     = std::string { target_color };
    config["binary_threshold"] = binary_threshold;
    config["min_contour_area"] = min_contour_area;
    config["max_contour_area"] = max_contour_area;
    return config;
}

auto make_feature_config(double min_area = 10.0, double max_area = 1500.0,
    double min_side_ratio = 0.2, double max_side_ratio = 2.5, double min_roundness = 0.1,
    double max_roundness = 0.9, double max_sub_area_ratio = 0.5, double min_convex_area_ratio = 0.3,
    double max_defect_area_ratio = 0.3, double min_area_for_ratio = 70.0,
    double center_concentricity_ratio = 0.08) -> YAML::Node {
    auto config                                    = YAML::Node {};
    config["center"]["min_area"]                   = min_area;
    config["center"]["max_area"]                   = max_area;
    config["center"]["min_side_ratio"]             = min_side_ratio;
    config["center"]["max_side_ratio"]             = max_side_ratio;
    config["center"]["min_roundness"]              = min_roundness;
    config["center"]["max_roundness"]              = max_roundness;
    config["center"]["max_sub_area_ratio"]         = max_sub_area_ratio;
    config["center"]["min_convex_area_ratio"]      = min_convex_area_ratio;
    config["center"]["max_defect_area_ratio"]      = max_defect_area_ratio;
    config["center"]["min_area_for_ratio"]         = min_area_for_ratio;
    config["center"]["center_concentricity_ratio"] = center_concentricity_ratio;
    return config;
}

auto make_bgr_image(int rows, int cols, const cv::Scalar& color = cv::Scalar::all(0))
    -> std::unique_ptr<Image> {
    auto image           = std::make_unique<Image>();
    image->details().mat = cv::Mat(rows, cols, CV_8UC3, color).clone();
    return image;
}

TEST(RuneFeatureIdentifier, InitializeAcceptsValidCenterConfig) {
    auto identifier = identifier::RuneFeatureIdentifier {};
    auto result     = identifier.initialize(make_feature_config());

    ASSERT_TRUE(result.has_value()) << result.error();
}

TEST(RuneFeatureIdentifier, InitializeRejectsInvalidAreaRange) {
    auto identifier = identifier::RuneFeatureIdentifier {};
    auto result     = identifier.initialize(make_feature_config(100.0, 10.0));

    ASSERT_FALSE(result.has_value());
    EXPECT_NE(result.error().find("center.max_area"), std::string::npos);
}

TEST(RuneFeatureIdentifier, InitializeRejectsInvalidSideRatioRange) {
    auto identifier = identifier::RuneFeatureIdentifier {};
    auto result     = identifier.initialize(make_feature_config(50.0, 10000.0, 2.0, 1.0));

    ASSERT_FALSE(result.has_value());
    EXPECT_NE(result.error().find("center.max_side_ratio"), std::string::npos);
}

TEST(RuneFeatureIdentifier, SyncIdentifyRejectsUninitializedIdentifier) {
    auto identifier = identifier::RuneFeatureIdentifier {};
    auto result     = identifier.sync_identify(identifier::RunePreprocessResult {});

    ASSERT_FALSE(result.has_value());
    EXPECT_NE(result.error().find("not initialized"), std::string::npos);
}

TEST(RuneFeatureIdentifier, SyncIdentifyReturnsEmptyCentersWhenNoCandidates) {
    auto identifier = identifier::RuneFeatureIdentifier {};
    ASSERT_TRUE(identifier.initialize(make_feature_config()).has_value());

    auto result = identifier.sync_identify(identifier::RunePreprocessResult {});

    ASSERT_TRUE(result.has_value());
    EXPECT_TRUE(result->centers.empty());
}

TEST(RuneFeatureIdentifier, SyncIdentifyRejectsMismatchedHierarchySize) {
    auto identifier = identifier::RuneFeatureIdentifier {};
    ASSERT_TRUE(identifier.initialize(make_feature_config()).has_value());

    auto bad_result = identifier::RunePreprocessResult {};
    bad_result.candidates.push_back(identifier::RuneContourCandidate {});

    auto result = identifier.sync_identify(bad_result);

    ASSERT_FALSE(result.has_value());
    EXPECT_NE(result.error().find("size mismatch"), std::string::npos);
}

TEST(RuneFeatureIdentifier, SyncIdentifyFindsCenterFromValidHierarchy) {
    auto preprocessor = identifier::RunePreprocessor {};
    ASSERT_TRUE(
        preprocessor.initialize(make_preprocessor_config("red", 1, 20.0, 10000.0)).has_value());

    auto identifier = identifier::RuneFeatureIdentifier {};
    ASSERT_TRUE(identifier.initialize(make_feature_config()).has_value());

    auto image = make_bgr_image(160, 160);
    cv::circle(image->details().mat, cv::Point(60, 80), 22, cv::Scalar(0, 0, 255), cv::FILLED);
    cv::circle(image->details().mat, cv::Point(68, 80), 6, cv::Scalar::all(0), cv::FILLED);
    cv::rectangle(
        image->details().mat, cv::Rect(105, 65, 45, 14), cv::Scalar(0, 0, 255), cv::FILLED);

    auto preprocess_result = preprocessor.sync_process(*image);
    ASSERT_TRUE(preprocess_result.has_value());

    auto result = identifier.sync_identify(*preprocess_result);

    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result->centers.size(), 1U);
    EXPECT_NEAR(result->centers.front().center.x, 60.0F, 3.0F);
    EXPECT_NEAR(result->centers.front().center.y, 80.0F, 3.0F);
    EXPECT_GT(result->centers.front().area, 1000.0);
}

TEST(RuneFeatureIdentifier, SyncIdentifyRejectsNonCenterLikeContour) {
    auto preprocessor = identifier::RunePreprocessor {};
    ASSERT_TRUE(
        preprocessor.initialize(make_preprocessor_config("red", 1, 20.0, 10000.0)).has_value());

    auto identifier = identifier::RuneFeatureIdentifier {};
    ASSERT_TRUE(identifier.initialize(make_feature_config()).has_value());

    auto image = make_bgr_image(140, 180);
    cv::rectangle(
        image->details().mat, cv::Rect(10, 40, 90, 24), cv::Scalar(0, 0, 255), cv::FILLED);
    cv::rectangle(
        image->details().mat, cv::Rect(115, 35, 50, 14), cv::Scalar(0, 0, 255), cv::FILLED);

    auto preprocess_result = preprocessor.sync_process(*image);
    ASSERT_TRUE(preprocess_result.has_value());

    auto result = identifier.sync_identify(*preprocess_result);

    ASSERT_TRUE(result.has_value());
    EXPECT_TRUE(result->centers.empty());
}

TEST(RuneFeatureIdentifier, SyncIdentifyRejectsIsolatedRootContour) {
    auto preprocessor = identifier::RunePreprocessor {};
    ASSERT_TRUE(
        preprocessor.initialize(make_preprocessor_config("red", 1, 20.0, 10000.0)).has_value());

    auto identifier = identifier::RuneFeatureIdentifier {};
    ASSERT_TRUE(identifier.initialize(make_feature_config()).has_value());

    auto image = make_bgr_image(120, 120);
    cv::circle(image->details().mat, cv::Point(60, 60), 18, cv::Scalar(0, 0, 255), cv::FILLED);

    auto preprocess_result = preprocessor.sync_process(*image);
    ASSERT_TRUE(preprocess_result.has_value());

    auto result = identifier.sync_identify(*preprocess_result);

    ASSERT_TRUE(result.has_value());
    EXPECT_TRUE(result->centers.empty());
}

} // namespace
