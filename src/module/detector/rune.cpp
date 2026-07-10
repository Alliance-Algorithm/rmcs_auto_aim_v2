#include "rune.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>
#include <ranges>
#include <string_view>
#include <vector>

namespace rmcs {

namespace details {
    constexpr auto kBullseyeRadius = 0.15;
    constexpr auto kRuneIconRadius = 0.05;

    auto compute_distance_with_icon(const std::vector<cv::Point>& contour) -> double {
        constexpr auto kBinsR     = 5;
        constexpr auto kBinsTheta = 12;
        constexpr auto kN         = 100;

        using Hist = std::array<float, kBinsR * kBinsTheta>;

        static const auto kTemplateContour = [] {
            constexpr auto kTemplate = std::array {
                std::string_view { " ###################    " },
                std::string_view { " ####################   " },
                std::string_view { "  ####################  " },
                std::string_view { "   #################### " },
                std::string_view { "   #################### " },
                std::string_view { "    #######       ######" },
                std::string_view { "     #######       #####" },
                std::string_view { "     #######       #####" },
                std::string_view { "      #######      #####" },
                std::string_view { "   #################### " },
                std::string_view { "   #################### " },
                std::string_view { "   ###################  " },
                std::string_view { "  ###################   " },
                std::string_view { "  #################     " },
                std::string_view { "  ######    #######     " },
                std::string_view { "  ######     ######     " },
                std::string_view { " ######      #######    " },
                std::string_view { " ######       #######   " },
                std::string_view { " ######        #######  " },
                std::string_view { " ######    ############ " },
                std::string_view { "######      ########### " },
                std::string_view { "######        #####     " },
                std::string_view { "                ####    " },
                std::string_view { "                  ###   " },
                std::string_view { "                    ##  " },
                std::string_view { "                      # " },
            };

            const auto rows = static_cast<int>(kTemplate.size());
            const auto cols = static_cast<int>(kTemplate[0].size());

            auto mat = cv::Mat(rows, cols, CV_8UC1, cv::Scalar { 0 });
            for (const auto [r, row] : kTemplate | std::views::enumerate) {
                for (const auto [c, ch] : row | std::views::enumerate) {
                    if (ch == '#')
                        mat.at<std::uint8_t>(static_cast<int>(r), static_cast<int>(c)) = 255;
                }
            }

            auto contours = std::vector<std::vector<cv::Point>> { };
            cv::findContours(mat, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            if (contours.empty()) return std::vector<cv::Point> { };

            return *std::ranges::max_element(
                contours, { }, [](const auto& c) { return cv::contourArea(c); });
        }();

        auto resample = [](const std::vector<cv::Point>& c,
                            std::size_t n) -> std::vector<cv::Point2f> {
            if (c.size() < 2 || n < 2) return { };

            auto seg_len = std::vector<double> { };
            seg_len.reserve(c.size());
            auto total = 0.0;
            for (std::size_t i = 1; i < c.size(); ++i) {
                const auto dx = static_cast<double>(c[i].x - c[i - 1].x);
                const auto dy = static_cast<double>(c[i].y - c[i - 1].y);
                seg_len.push_back(std::hypot(dx, dy));
                total += seg_len.back();
            }
            {
                const auto dx = static_cast<double>(c[0].x - c.back().x);
                const auto dy = static_cast<double>(c[0].y - c.back().y);
                seg_len.push_back(std::hypot(dx, dy));
                total += seg_len.back();
            }

            auto sampled = std::vector<cv::Point2f> { };
            sampled.reserve(n);
            const auto step  = total / static_cast<double>(n);
            const auto seg_n = seg_len.size();
            auto acc         = 0.0;
            auto seg_idx     = std::size_t { 0 };

            for (std::size_t i = 0; i < n; ++i) {
                const auto target = static_cast<double>(i) * step;
                while (seg_idx < seg_n && acc + seg_len[seg_idx] < target) {
                    acc += seg_len[seg_idx];
                    ++seg_idx;
                }
                if (seg_idx >= seg_n) seg_idx = seg_n - 1;
                const auto alpha = seg_len[seg_idx] > 0.0 ? (target - acc) / seg_len[seg_idx] : 0.0;
                const auto& p0   = c[seg_idx % c.size()];
                const auto& p1   = c[(seg_idx + 1) % c.size()];
                sampled.emplace_back(static_cast<float>(p0.x + alpha * (p1.x - p0.x)),
                    static_cast<float>(p0.y + alpha * (p1.y - p0.y)));
            }

            return sampled;
        };

        static const auto kLogDistRange = [&] {
            const auto sampled  = resample(kTemplateContour, kN);
            constexpr auto kEps = 1e-6;

            auto all_dists = std::vector<double> { };
            all_dists.reserve(sampled.size() * sampled.size() / 2);
            for (std::size_t i = 0; i < sampled.size(); ++i) {
                for (std::size_t j = i + 1; j < sampled.size(); ++j) {
                    const auto dx = static_cast<double>(sampled[j].x - sampled[i].x);
                    const auto dy = static_cast<double>(sampled[j].y - sampled[i].y);
                    all_dists.push_back(std::log(std::sqrt(dx * dx + dy * dy) + kEps));
                }
            }

            const auto [r_min, r_max] = std::ranges::minmax(all_dists);
            auto range                = r_max - r_min;
            if (range < 1.0) range = 1.0;
            return std::pair { r_min - 0.2 * range, r_max + 0.2 * range };
        }();

        const auto [global_r_min, global_r_max] = kLogDistRange;
        auto global_r_range                     = global_r_max - global_r_min;
        if (global_r_range < 1e-3) global_r_range = 1.0;

        auto compute_hist = [&](const std::vector<cv::Point>& c) -> Hist {
            const auto sampled = resample(c, kN);
            if (sampled.size() != kN) return { };

            auto hist            = Hist { };
            constexpr auto n_r   = static_cast<double>(kBinsR);
            constexpr auto n_t   = static_cast<double>(kBinsTheta);
            constexpr auto pi    = std::numbers::pi;
            constexpr auto twopi = 2.0 * pi;

            for (std::size_t i = 0; i < kN; ++i) {
                auto log_dists = std::vector<double> { };
                auto angles    = std::vector<double> { };
                log_dists.reserve(kN - 1);
                angles.reserve(kN - 1);
                for (std::size_t j = 0; j < kN; ++j) {
                    if (i == j) continue;
                    const auto dx = static_cast<double>(sampled[j].x - sampled[i].x);
                    const auto dy = static_cast<double>(sampled[j].y - sampled[i].y);
                    log_dists.push_back(std::log(std::sqrt(dx * dx + dy * dy) + 1e-6));
                    angles.push_back(std::atan2(dy, dx));
                }

                for (std::size_t k = 0; k < log_dists.size(); ++k) {
                    const auto r_bin = std::clamp(
                        static_cast<int>((log_dists[k] - global_r_min) / global_r_range * n_r), 0,
                        kBinsR - 1);
                    const auto theta_bin = std::clamp(
                        static_cast<int>((angles[k] + pi) / twopi * n_t), 0, kBinsTheta - 1);
                    hist[static_cast<std::size_t>(r_bin) * kBinsTheta
                        + static_cast<std::size_t>(theta_bin)] += 1.0f;
                }
            }

            for (auto& h : hist)
                h /= static_cast<float>(kN);
            auto sum = 0.0f;
            for (const auto& h : hist)
                sum += h;
            if (sum > 0.0f)
                for (auto& h : hist)
                    h /= sum;

            return hist;
        };
        static const auto kTemplateSc = compute_hist(kTemplateContour);

        const auto sc = compute_hist(contour);

        auto dist = 0.0;
        for (std::size_t i = 0; i < sc.size(); ++i) {
            const auto diff = static_cast<double>(sc[i]) - static_cast<double>(kTemplateSc[i]);
            const auto sum =
                static_cast<double>(sc[i]) + static_cast<double>(kTemplateSc[i]) + 1e-10;
            dist += diff * diff / sum;
        }

        return dist;
    }

    auto compute_bullseye_feature(const cv::Mat& roi, cv::Point2f center,
        std::array<cv::Point2f, 4>& endpoints, double& score, double active_threshold) -> bool {

        constexpr auto kRadialBins         = 32;
        constexpr auto kThetaBins          = 180;
        constexpr auto kArmLengthStddevMax = 0.3;

        auto square = cv::Mat { };
        {
            const auto square_size = std::max(roi.cols, roi.rows);

            square = cv::Mat(square_size, square_size, roi.type(), cv::Scalar { 0, 0, 0 });
            roi.copyTo(square(cv::Rect { 0, 0, roi.cols, roi.rows }));
        }

        auto gray = cv::Mat { };
        if (square.channels() == 3) {
            cv::cvtColor(square, gray, cv::COLOR_BGR2GRAY);
        } else {
            gray = square;
        }

        const auto max_radius = std::min(square.cols, square.rows) * 0.5;

        auto polar_hist = std::array<float, static_cast<std::size_t>(kRadialBins * kThetaBins)> { };
        {
            for (int y = 0; y < gray.rows; ++y) {
                for (int x = 0; x < gray.cols; ++x) {
                    const auto dx = static_cast<double>(x) - center.x;
                    const auto dy = static_cast<double>(y) - center.y;
                    const auto r  = std::hypot(dx, dy);
                    if (r >= max_radius) continue;

                    const auto theta = std::atan2(dy, dx);
                    const auto r_bin = std::clamp(
                        static_cast<int>(r / max_radius * kRadialBins), 0, kRadialBins - 1);
                    const auto t_bin = std::clamp<double>(
                        (theta + std::numbers::pi) / (2 * std::numbers::pi) * kThetaBins, 0,
                        kThetaBins - 1);

                    polar_hist[static_cast<int>(r_bin * kThetaBins + t_bin)] +=
                        static_cast<float>(gray.at<std::uint8_t>(y, x));
                }
            }
        }

        auto angular_proj = std::array<double, static_cast<std::size_t>(kThetaBins)> { };
        {
            for (int t = 0; t < kThetaBins; ++t) {
                for (int r = 0; r < kRadialBins; ++r) {
                    angular_proj[t] += polar_hist[r * kThetaBins + t] * static_cast<float>(r + 1);
                }
            }
        }
        auto sum_cos = 0.0;
        auto sum_sin = 0.0;
        auto sum_dc  = 0.0;
        for (int t = 0; t < kThetaBins; ++t) {
            const auto angle = 4.0 * static_cast<double>(t) / kThetaBins * 2.0 * std::numbers::pi;
            sum_cos += angular_proj[t] * std::cos(angle);
            sum_sin += angular_proj[t] * std::sin(angle);
            sum_dc += angular_proj[t];
        }
        const auto harmonic_ratio = std::hypot(sum_cos, sum_sin) / (sum_dc + 1e-10);
        score                     = harmonic_ratio;
        if (harmonic_ratio < active_threshold) {

            auto radial_total = std::array<double, static_cast<std::size_t>(kRadialBins)> { };
            for (int r = 0; r < kRadialBins; ++r) {
                for (int t = 0; t < kThetaBins; ++t) {
                    radial_total[r] += polar_hist[r * kThetaBins + t];
                }
            }

            const auto outer_radius_bin = static_cast<std::size_t>(
                std::ranges::max_element(radial_total) - radial_total.begin());
            const auto radius =
                max_radius * static_cast<double>(outer_radius_bin + 1) / kRadialBins;

            endpoints = {
                center + cv::Point2f { 0.0f, -static_cast<float>(radius) },
                center + cv::Point2f { +static_cast<float>(radius), 0.0f },
                center + cv::Point2f { 0.0f, +static_cast<float>(radius) },
                center + cv::Point2f { -static_cast<float>(radius), 0.0f },
            };
            return true;
        }

        const auto phase = std::atan2(sum_sin, sum_cos) / 4.0;

        auto tips = std::array<cv::Point2f, 4> { };
        {
            constexpr auto pi    = std::numbers::pi;
            constexpr auto twopi = 2.0 * pi;

            for (std::size_t p = 0; p < 4; ++p) {
                const auto theta = phase + static_cast<double>(p) * pi * 0.5;
                const auto t_idx =
                    static_cast<int>(std::llround((theta + pi) / twopi * kThetaBins)) % kThetaBins;
                const auto dx = std::cos(theta);
                const auto dy = std::sin(theta);

                auto radial_profile = std::array<double, static_cast<std::size_t>(kRadialBins)> { };
                for (int r = 0; r < kRadialBins; ++r) {
                    for (int dt = -3; dt <= 3; ++dt)
                        radial_profile[r] +=
                            polar_hist[r * kThetaBins + ((t_idx + dt + kThetaBins) % kThetaBins)];
                }

                const auto max_energy = *std::ranges::max_element(radial_profile);
                const auto threshold  = max_energy * 0.25;

                auto boundary = -1;
                for (int r = kRadialBins - 1; r >= 0; --r) {
                    if (radial_profile[r] >= threshold) {
                        boundary = r;
                        break;
                    }
                }

                auto exact_r = 0.0;
                if (boundary >= kRadialBins - 1) {
                    exact_r = 1.0;
                } else if (boundary < 0) {
                    exact_r = static_cast<double>(
                                  std::ranges::max_element(radial_profile) - radial_profile.begin())
                        / kRadialBins;
                } else {
                    const auto e_in  = radial_profile[boundary];
                    const auto e_out = radial_profile[boundary + 1];
                    if (e_in > e_out) {
                        const auto frac = (threshold - e_out) / (e_in - e_out);
                        exact_r         = (static_cast<double>(boundary + 1) - frac) / kRadialBins;
                    } else {
                        exact_r = static_cast<double>(boundary + 1) / kRadialBins;
                    }
                }

                const auto arm_len = max_radius * exact_r;
                tips[p]            = cv::Point2f {
                    center.x + static_cast<float>(arm_len * dx),
                    center.y + static_cast<float>(arm_len * dy),
                };
            }
        }

        {
            auto lengths = std::array<double, 4> { };
            auto mean    = 0.0;
            for (std::size_t i = 0; i < 4; ++i) {
                lengths[i] = std::hypot(tips[i].x - center.x, tips[i].y - center.y);
                mean += lengths[i];
            }
            mean *= 0.25;

            auto variance = 0.0;
            for (const auto l : lengths) {
                const auto diff = l - mean;
                variance += diff * diff;
            }
            variance *= 0.25;
            if (std::sqrt(variance) > mean * kArmLengthStddevMax) return false;
        }

        score     = harmonic_ratio;
        endpoints = tips;
        return true;
    }

    auto find_contours(const RuneDetector::Config& config, const cv::Mat& mat,
        std::vector<std::vector<cv::Point>>& icons,
        std::vector<std::vector<cv::Point>>& bullseyes) noexcept {

        icons.clear();
        bullseyes.clear();
        if (mat.empty() || mat.type() != CV_8UC1) return;

        const auto fx = config.cam.camera_matrix[0][0];
        const auto fy = config.cam.camera_matrix[1][1];
        if (fx <= 0.0 || fy <= 0.0) return;
        const auto focal = (fx + fy) * 0.5;

        const auto perspective_rad = config.max_perspective * std::numbers::pi / 180.0;
        const auto cos_theta       = std::cos(perspective_rad);
        if (cos_theta <= 0.0) return;

        const auto radius_range = [=](double physical_radius, double min_distance,
                                      double max_distance) -> std::array<double, 2> {
            constexpr auto kMargin = 0.1;

            const auto r_min = focal * physical_radius * cos_theta / max_distance * (1.0 - kMargin);
            const auto r_max =
                focal * physical_radius / (min_distance * cos_theta) * (1.0 + kMargin);

            return { r_min, r_max };
        };
        const auto area_range = [=](double physical_radius, double min_distance,
                                    double max_distance,
                                    double fill_ratio) -> std::array<double, 2> {
            constexpr auto kMargin = 0.1;

            const auto a_min = focal * focal * physical_radius * physical_radius * cos_theta
                / (max_distance * max_distance) * (1.0 - kMargin) * std::numbers::pi * fill_ratio;
            const auto a_max = focal * focal * physical_radius * physical_radius
                / (min_distance * min_distance * cos_theta) * (1.0 + kMargin) * std::numbers::pi;

            return { a_min, a_max };
        };

        const auto bullseye_range =
            radius_range(kBullseyeRadius, config.min_distance, config.max_distance);
        const auto icon_area_range =
            area_range(kRuneIconRadius, config.min_distance, config.max_distance, 0.2);

        const auto min_circularity = 2.0 * cos_theta / (1.0 + cos_theta * cos_theta);

        auto contours  = std::vector<std::vector<cv::Point>> { };
        auto hierarchy = std::vector<cv::Vec4i> { };
        cv::findContours(mat.clone(), contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

        struct Ellipse {
            cv::Point center;
            double major;
            double minor;
            std::size_t contour_index;
        };
        auto visited = std::vector<Ellipse> { };

        for (const auto& [i, contour] : contours | std::views::enumerate) {
            if (contour.size() < 5) continue;

            const auto area = cv::contourArea(contour);
            if (area <= 0.0) continue;

            const auto perimeter = cv::arcLength(contour, true);
            if (perimeter <= 0.0) continue;

            const auto radius = std::sqrt(area / std::numbers::pi);

            /*^^*/ if (radius >= bullseye_range[0] && radius <= bullseye_range[1]) {
                constexpr auto kEllipseAreaRatioMin = 0.7;
                constexpr auto kEllipseAreaRatioMax = 1.3;
                constexpr auto kCircularityMargin   = 0.7;

                const auto ellipse = cv::fitEllipse(contour);
                const auto major   = std::max(ellipse.size.width, ellipse.size.height);
                const auto minor   = std::min(ellipse.size.width, ellipse.size.height);
                if (major <= 0.0 || minor <= 0.0) continue;

                const auto aspect_ratio = minor / major;
                if (aspect_ratio < cos_theta) continue;

                const auto ellipse_area = std::numbers::pi * (major * 0.5) * (minor * 0.5);
                const auto area_ratio   = area / ellipse_area;

                if (area_ratio < kEllipseAreaRatioMin || area_ratio > kEllipseAreaRatioMax)
                    continue;

                const auto circularity = 4.0 * std::numbers::pi * area / (perimeter * perimeter);
                if (circularity < min_circularity * kCircularityMargin) continue;

                const auto moment = cv::moments(contour);
                visited.push_back(Ellipse {
                    .center =
                        cv::Point {
                            static_cast<int>(moment.m10 / moment.m00),
                            static_cast<int>(moment.m01 / moment.m00),
                        },
                    .major = static_cast<double>(major),
                    .minor = static_cast<double>(minor),

                    .contour_index = static_cast<std::size_t>(i),
                });
            } else if (area >= icon_area_range[0] && area <= icon_area_range[1]) {
                const auto ellipse = cv::fitEllipse(contour);
                const auto major   = std::max(ellipse.size.width, ellipse.size.height);
                const auto minor   = std::min(ellipse.size.width, ellipse.size.height);
                if (major <= 0.0 || minor <= 0.0) continue;

                const auto aspect_ratio = minor / major;
                if (aspect_ratio < cos_theta) continue;

                icons.push_back(contour);
            }
        }

        {
            constexpr auto kConcentricTolerance = 0.3;

            auto seen = std::vector<bool>(visited.size(), false);

            auto indices = std::views::iota(std::size_t { 0 }, visited.size())
                | std::views::filter([&](std::size_t n) { return !seen[n]; });

            for (auto i : indices) {
                auto group = std::vector<std::size_t> { i };
                seen[i]    = true;

                auto concentric = std::views::iota(i + 1, visited.size())
                    | std::views::filter([&](std::size_t j) {
                          if (seen[j]) return false;
                          const auto& a   = visited[i];
                          const auto& b   = visited[j];
                          const auto dist = std::hypot(static_cast<double>(a.center.x - b.center.x),
                              static_cast<double>(a.center.y - b.center.y));
                          return dist < std::max(a.major, b.major) * 0.5 * kConcentricTolerance;
                      });

                for (auto j : concentric) {
                    group.push_back(j);
                    seen[j] = true;
                }

                if (group.empty()) continue;

                const auto largest =
                    std::ranges::max(group, { }, [&](std::size_t n) { return visited[n].major; });
                bullseyes.push_back(contours[visited[largest].contour_index]);
            }
        }

        auto filtered_icons = std::vector<std::vector<cv::Point>> { };
        std::ranges::copy(icons | std::views::filter([&bullseyes](const auto& icon) {
            const auto moment = cv::moments(icon);
            const auto center = cv::Point2f {
                static_cast<float>(moment.m10 / moment.m00),
                static_cast<float>(moment.m01 / moment.m00),
            };

            return !std::ranges::any_of(bullseyes, [&center](const auto& bull) {
                return cv::pointPolygonTest(bull, center, false) >= 0;
            });
        }),
            std::back_inserter(filtered_icons));

        icons = std::move(filtered_icons);
    }

}

auto RuneDetector::detect(const cv::Mat& mat) const -> Elements {
    if (mat.empty()) return { };

    auto binary = cv::Mat { };
    {
        auto channels = std::vector<cv::Mat> { };
        cv::split(mat, channels);

        auto difference    = cv::Mat { };
        auto difference_u8 = cv::Mat { };
        auto minimum_mask  = cv::Mat { };
        auto mask          = cv::Mat { };

        /*^^*/ if (config.color == CampColor::RED) {
            cv::subtract(channels[2], channels[0], difference, cv::noArray(), CV_16S);
            difference.convertTo(difference_u8, CV_8U);
            cv::threshold(difference_u8, mask, config.red_diff_threshold, 255, cv::THRESH_BINARY);

            cv::threshold(
                channels[2], minimum_mask, config.min_channel_threshold, 255, cv::THRESH_BINARY);
        } else if (config.color == CampColor::BLUE) {
            cv::subtract(channels[0], channels[2], difference, cv::noArray(), CV_16S);
            difference.convertTo(difference_u8, CV_8U);
            cv::threshold(difference_u8, mask, config.blue_diff_threshold, 255, cv::THRESH_BINARY);

            cv::threshold(
                channels[0], minimum_mask, config.min_channel_threshold, 255, cv::THRESH_BINARY);
        } else {
            return { };
        }

        cv::bitwise_and(mask, minimum_mask, binary);

        auto kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size { 5, 5 });
        cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, kernel);
    }

    auto icons     = std::vector<std::vector<cv::Point>> { };
    auto bullseyes = std::vector<std::vector<cv::Point>> { };
    details::find_contours(config, binary, icons, bullseyes);

    auto result = Elements { };

    for (const auto& bull : bullseyes) {
        const auto br          = cv::boundingRect(bull);
        constexpr auto kMargin = 20;
        auto roi               = cv::Rect {
            br.x - kMargin,
            br.y - kMargin,
            br.width + 2 * kMargin,
            br.height + 2 * kMargin,
        };
        roi &= cv::Rect { 0, 0, mat.cols, mat.rows };
        if (roi.width <= 0 || roi.height <= 0) continue;

        const auto moment     = cv::moments(bull);
        const auto center_roi = cv::Point2f {
            static_cast<float>(moment.m10 / moment.m00 - roi.x),
            static_cast<float>(moment.m01 / moment.m00 - roi.y),
        };

        auto masked_roi = cv::Mat { };
        {
            auto contour_local = bull;
            for (auto& p : contour_local) {
                p.x -= roi.x;
                p.y -= roi.y;
            }

            auto mask = cv::Mat { };
            mask      = cv::Mat::zeros(roi.size(), CV_8UC1);
            auto ctns = std::vector<std::vector<cv::Point>> { contour_local };
            cv::drawContours(mask, ctns, -1, cv::Scalar { 255 }, cv::FILLED);
            mat(roi).copyTo(masked_roi, mask);
        }

        auto endpoints = std::array<cv::Point2f, 4> { };
        auto score     = double { };
        if (!details::compute_bullseye_feature(
                masked_roi, center_roi, endpoints, score, config.active_threshold))
            continue;

        for (auto& p : endpoints) {
            p.x += static_cast<float>(roi.x);
            p.y += static_cast<float>(roi.y);
        }

        result.bullseyes.push_back({
            .center =
                Point2d {
                    center_roi.x + static_cast<float>(roi.x),
                    center_roi.y + static_cast<float>(roi.y),
                },
            .corners = { {
                Point2d { endpoints[0] },
                Point2d { endpoints[1] },
                Point2d { endpoints[2] },
                Point2d { endpoints[3] },
            } },
            .active  = score < config.active_threshold,
            .score   = score,
        });
    }

    for (const auto& icon : icons) {
        const auto score = details::compute_distance_with_icon(icon);
        if (score >= config.match_threshold) continue;

        const auto rc = cv::minAreaRect(icon);
        result.icons.emplace_back(Point2d { rc.center }, score);
    }

    return result;
}

} // namespace rmcs
