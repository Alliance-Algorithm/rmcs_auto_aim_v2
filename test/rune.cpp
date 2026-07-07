#include "module/detector/rune.hpp"

#include <algorithm>
#include <array>
#include <cmath>

#include <gtest/gtest.h>
#include <opencv2/imgcodecs.hpp>

TEST(rune, detect_two_inactive_pages) {
    constexpr auto image_path = "/workspaces/data/autoaim/image/红色大符.jpg";
    auto image                = cv::imread(image_path, cv::IMREAD_COLOR);
    ASSERT_FALSE(image.empty()) << "Failed to load image: " << image_path;

    auto finder        = rmcs::RuneFinder { };
    finder.input.image = image;
    finder.input.color = rmcs::CampColor::RED;

    ASSERT_TRUE(finder.solve()) << "RuneFinder did not detect any pages";

    // 验证 R 标
    EXPECT_FALSE(finder.result.icon.x == 0.0 && finder.result.icon.y == 0.0) //
        << "Rune icon (center) was not detected";

    // 统计并验证页数量
    std::size_t active_count   = 0;
    std::size_t inactive_count = 0;
    for (const auto& page : finder.result.pages) {
        if (page.active) {
            ++active_count;
            continue;
        }

        ++inactive_count;
        EXPECT_EQ(page.gap_valid.size(), 4);
        EXPECT_EQ(page.gap_corners.size(), 4);
    }
    EXPECT_EQ(active_count, 0) << "Unexpected active pages detected";
    EXPECT_EQ(inactive_count, 2) << "Expected exactly 2 inactive pages";

    // 验证中心位置：允许 30 像素误差
    constexpr auto kCenterTolerance = 30.0;

    EXPECT_NEAR(finder.result.icon.x, 750.0, kCenterTolerance);
    EXPECT_NEAR(finder.result.icon.y, 450.0, kCenterTolerance);

    constexpr auto kExpectedCenters = std::array<rmcs::Point2d, 2> {
        rmcs::Point2d { 520.0, 480.0 },
        rmcs::Point2d { 900.0, 325.0 },
    };

    for (const auto& expected : kExpectedCenters) {
        const auto found = std::ranges::any_of(finder.result.pages, [&](const auto& page) {
            if (page.active) return false;

            const auto dx = std::abs(page.center.x - expected.x);
            const auto dy = std::abs(page.center.y - expected.y);
            return dx < kCenterTolerance && dy < kCenterTolerance;
        });

        EXPECT_TRUE(found) << "Expected inactive page center near (" << expected.x << ", "
                           << expected.y << ") not found";
    }
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
