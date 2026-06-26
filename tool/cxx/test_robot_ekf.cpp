#include "module/predictor/model/robot.hpp"
#include "utility/math/angle.hpp"
#include "utility/math/camera.hpp"
#include "utility/math/reprojection.hpp"
#include "utility/math/robot.hpp"
#include "utility/rclcpp/node.details.hpp"
#include "utility/robot/armor.hpp"

#include <algorithm>
#include <array>
#include <numbers>
#include <print>
#include <random>
#include <string_view>

#include <eigen3/Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using namespace rmcs;
using namespace rmcs::util;

namespace {

struct MockMotion {
    static constexpr auto kRotationSpeed    = 1 * std::numbers::pi;
    static constexpr auto kAmplitude        = 2.0;
    static constexpr auto kPeriod           = 20.0;
    static constexpr auto kCycleDuration    = 15.0;
    inline static const auto kInitialCenter = Eigen::Vector3d { 3.0, 0.0, 0.0 };

    static auto position(double elapsed) -> Eigen::Vector3d {
        auto center = kInitialCenter;
        center.y() += kAmplitude * std::sin(2.0 * std::numbers::pi * elapsed / kPeriod);
        return center;
    }

    static auto rotation_angle(double elapsed) -> double {
        const auto cycle_elapsed = std::fmod(elapsed, kCycleDuration);
        const auto phase         = cycle_elapsed < 5.0 ? 0 : cycle_elapsed < 10.0 ? 1 : 2;
        const auto cycles_done   = static_cast<int>(elapsed / kCycleDuration);

        auto rotate_time = cycles_done * 10.0;
        if (phase == 0) rotate_time += cycle_elapsed;
        else if (phase == 1) rotate_time += 5.0;
        else rotate_time += 5.0 + (cycle_elapsed - 10.0);

        return kRotationSpeed * rotate_time;
    }
};

struct MockScene {
    static constexpr auto kRadiusForward = 0.2;
    static constexpr auto kRadiusLateral = 0.18;
    static constexpr auto kHeightLateral = 0.02;

    const Eigen::Vector3d center;
    const double theta;

    std::array<Armor3d, 4> armors;
    std::array<Lightbar3d, 8> lightbars;

    MockScene(const Eigen::Vector3d& center, double theta) noexcept
        : center { center }
        , theta { theta } {
        auto solution                 = RobotSolution { };
        solution.input.center         = Translation { center };
        solution.input.toward         = Orientation { Eigen::Quaterniond {
            Eigen::AngleAxisd { theta, Eigen::Vector3d::UnitZ() } } };
        solution.input.radius_forward = kRadiusForward;
        solution.input.radius_lateral = kRadiusLateral;
        solution.input.height_lateral = kHeightLateral;
        solution.input.genre          = DeviceId::INFANTRY_3;
        solution.input.color          = ArmorColor::BLUE;

        armors    = solution.solve_armors();
        lightbars = solution.solve_lightbars();
    }
};

struct Projector {
    static constexpr auto kImageWidth  = 1440;
    static constexpr auto kImageHeight = 1080;
    static constexpr auto kFx          = 1000.0;
    static constexpr auto kFy          = 1000.0;
    static constexpr auto kCx          = 720.0;
    static constexpr auto kCy          = 540.0;
    static constexpr auto kYawFullMax  = 60.0 * std::numbers::pi / 180.0;
    static constexpr auto kYawPartMax  = 90.0 * std::numbers::pi / 180.0;
    static constexpr auto kLineWidth   = 2;
    static constexpr auto kPointSize   = 3;

    CameraFeature camera;
    std::mt19937& pixel_engine;
    std::normal_distribution<double>& pixel_noise;

    Projector(std::mt19937& eng, std::normal_distribution<double>& noise) noexcept
        : pixel_engine { eng }
        , pixel_noise { noise } {
        camera.camera_matrix = { std::array { kFx, 0.0, kCx }, std::array { 0.0, kFy, kCy },
            std::array { 0.0, 0.0, 1.0 } };
        camera.distort_coeff = { 0.0, 0.0, 0.0, 0.0, 0.0 };
        camera.orientation   = Orientation::kIdentity();
        camera.translation   = Translation::kZero();
    }

    static auto compute_visibility(const MockScene& scene) {
        auto vis_parts = std::array<std::string_view, 4> {
            kNameNone,
            kNameNone,
            kNameNone,
            kNameNone,
        };

        for (int id = 0; id < 4; ++id) {
            const auto orientation = scene.armors[id].orientation.make<Eigen::Quaterniond>();
            const auto position    = scene.armors[id].translation.make<Eigen::Vector3d>();

            const auto armor_to_center = orientation * Eigen::Vector3d::UnitX();
            const auto yaw =
                std::acos(std::clamp((-armor_to_center).dot((-position).normalized()), -1.0, 1.0));

            if (yaw >= kYawPartMax) continue;

            vis_parts[id] = yaw < kYawFullMax ? kNameFull : kNamePart;
        }

        return vis_parts;
    }

    auto draw(const MockScene& scene, cv::Mat& img) -> void {
        const auto vis = compute_visibility(scene);
        for (int id = 0; id < 4; ++id) {
            if (vis[id][0] == '-') continue;
            const auto orientation = scene.armors[id].orientation.make<Eigen::Quaterniond>();
            const auto position    = scene.armors[id].translation.make<Eigen::Vector3d>();
            if (vis[id][0] == 'f') draw_full_armor(img, id, scene, position, orientation);
            else draw_partial_lightbar(img, id, scene, position, orientation);
        }
    }

private:
    inline static const auto kArmorColors = std::array<cv::Scalar, 4> {
        cv::Scalar { 255, 0, 0 }, // px
        cv::Scalar { 0, 255, 0 }, // py
        cv::Scalar { 0, 0, 255 }, // nx
        cv::Scalar { 255, 255, 0 }, // ny
    };
    static constexpr auto kNameFull = std::string_view { "full" };
    static constexpr auto kNamePart = std::string_view { "part" };
    static constexpr auto kNameNone = std::string_view { "-" };

    auto noisy_point(const Point2d& p) -> cv::Point {
        return cv::Point {
            static_cast<int>(p.x + pixel_noise(pixel_engine)),
            static_cast<int>(p.y + pixel_noise(pixel_engine)),
        };
    }

    auto project(const Point3d& p) const -> std::optional<Point2d> {
        return reproject_point(Point3d { -p.y, -p.z, p.x }, camera);
    }

    void draw_full_armor(cv::Mat& img, int id, const MockScene& scene,
        const Eigen::Vector3d& /*position*/, const Eigen::Quaterniond& /*orientation*/) {
        const auto l_bar = scene.lightbars[id * 2];
        const auto r_bar = scene.lightbars[id * 2 + 1];

        const auto tu = project(l_bar.upper);
        const auto tl = project(l_bar.lower);
        const auto bu = project(r_bar.upper);
        const auto bl = project(r_bar.lower);
        if (!tu || !tl || !bu || !bl) return;

        const auto& kC = kArmorColors[id];
        const auto& kD = kC * 0.5;

        cv::line(img, noisy_point(*tu), noisy_point(*tl), kC, kLineWidth);
        cv::line(img, noisy_point(*bu), noisy_point(*bl), kC, kLineWidth);
        cv::line(img, noisy_point(*tu), noisy_point(*bu), kD, 1);
        cv::line(img, noisy_point(*tl), noisy_point(*bl), kD, 1);
        cv::circle(img, noisy_point(*tu), kPointSize, kC, -1);
        cv::circle(img, noisy_point(*tl), kPointSize, kC, -1);
        cv::circle(img, noisy_point(*bu), kPointSize, kC, -1);
        cv::circle(img, noisy_point(*bl), kPointSize, kC, -1);
    }

    void draw_partial_lightbar(cv::Mat& img, int id, const MockScene& scene,
        const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation) {
        const auto& bar_l = scene.lightbars[id * 2];
        const auto& bar_r = scene.lightbars[id * 2 + 1];
        const auto dist_l = std::hypot(bar_l.upper.x, bar_l.upper.y, bar_l.upper.z);
        const auto dist_r = std::hypot(bar_r.upper.x, bar_r.upper.y, bar_r.upper.z);
        const auto bar    = dist_l < dist_r ? bar_l : bar_r;

        const auto up = project(bar.upper);
        const auto lo = project(bar.lower);
        if (!up || !lo) return;

        const auto kD = kArmorColors[id] * 0.5;
        cv::line(img, noisy_point(*up), noisy_point(*lo), kD, kLineWidth);
        cv::circle(img, noisy_point(*up), kPointSize, kD, -1);
        cv::circle(img, noisy_point(*lo), kPointSize, kD, -1);
    }
};

struct MarkerHelper {
    visualization_msgs::msg::MarkerArray array { };
    int index          = 0;
    rclcpp::Time stamp = { 0, 0, RCL_ROS_TIME };

    auto make(const std::string& ns) -> visualization_msgs::msg::Marker& {
        auto& m           = array.markers.emplace_back();
        m.header.frame_id = "odom_imu_link";
        m.header.stamp    = stamp;
        m.ns              = ns;
        m.id              = index++;
        m.action          = visualization_msgs::msg::Marker::ADD;
        m.lifetime        = rclcpp::Duration::from_seconds(0.1);
        return m;
    }
    auto add_cube(const std::string& ns, const Eigen::Vector3d& pos, const Eigen::Quaterniond& orn,
        double sx, double sy, double sz, const cv::Scalar& color) -> void {
        auto& m   = make(ns);
        m.type    = visualization_msgs::msg::Marker::CUBE;
        m.scale.x = static_cast<float>(sx);
        m.scale.y = static_cast<float>(sy);
        m.scale.z = static_cast<float>(sz);
        m.color.r = static_cast<float>(color[2] / 255.0);
        m.color.g = static_cast<float>(color[1] / 255.0);
        m.color.b = static_cast<float>(color[0] / 255.0);
        m.color.a = static_cast<float>(color[3] / 255.0);
        set_pose(m, pos, orn);
    }

    auto add_sphere(const std::string& ns, const Eigen::Vector3d& pos, double r,
        const cv::Scalar& color) -> void {
        auto& m   = make(ns);
        m.type    = visualization_msgs::msg::Marker::SPHERE;
        m.scale.x = static_cast<float>(r);
        m.scale.y = static_cast<float>(r);
        m.scale.z = static_cast<float>(r);
        m.color.r = static_cast<float>(color[2] / 255.0);
        m.color.g = static_cast<float>(color[1] / 255.0);
        m.color.b = static_cast<float>(color[0] / 255.0);
        m.color.a = static_cast<float>(color[3] / 255.0);
        set_pose(m, pos, Eigen::Quaterniond::Identity());
    }

    auto add_points(const std::string& ns, std::span<const Eigen::Vector3d> pts,
        const cv::Scalar& color) -> void {
        auto& m   = make(ns);
        m.type    = visualization_msgs::msg::Marker::POINTS;
        m.scale.x = 0.02f;
        m.scale.y = 0.02f;
        m.color.r = static_cast<float>(color[2] / 255.0);
        m.color.g = static_cast<float>(color[1] / 255.0);
        m.color.b = static_cast<float>(color[0] / 255.0);
        m.color.a = static_cast<float>(color[3] / 255.0);
        for (const auto& p : pts) {
            m.points.emplace_back(make_point(p));
        }
    }

    auto add_line_strip(const std::string& ns, std::span<const Eigen::Vector3d> pts, double width,
        const cv::Scalar& color) -> void {
        auto& m   = make(ns);
        m.type    = visualization_msgs::msg::Marker::LINE_STRIP;
        m.scale.x = static_cast<float>(width);
        m.color.r = static_cast<float>(color[2] / 255.0);
        m.color.g = static_cast<float>(color[1] / 255.0);
        m.color.b = static_cast<float>(color[0] / 255.0);
        m.color.a = static_cast<float>(color[3] / 255.0);
        for (const auto& p : pts) {
            m.points.emplace_back(make_point(p));
        }
    }

private:
    static auto make_point(const Eigen::Vector3d& p) -> geometry_msgs::msg::Point {
        auto pt = geometry_msgs::msg::Point { };
        pt.x    = p.x();
        pt.y    = p.y();
        pt.z    = p.z();
        return pt;
    }

    static auto set_pose(visualization_msgs::msg::Marker& m, const Eigen::Vector3d& pos,
        const Eigen::Quaterniond& orn) -> void {
        m.pose.position.x    = pos.x();
        m.pose.position.y    = pos.y();
        m.pose.position.z    = pos.z();
        m.pose.orientation.w = orn.w();
        m.pose.orientation.x = orn.x();
        m.pose.orientation.y = orn.y();
        m.pose.orientation.z = orn.z();
    }
};

} // namespace

auto main() -> int {
    rclcpp::init(0, nullptr);

    auto node = RclcppNode { "test_robot_ekf" };
    node.set_pub_topic_prefix("/rmcs/auto_aim/robot_test/");

    auto marker_pub = node.details->make_pub<visualization_msgs::msg::MarkerArray>(
        "/rmcs/auto_aim/robot_test/ekf", qos::debug);
    auto image_pub = node.details->make_pub<sensor_msgs::msg::Image>(
        "/rmcs/auto_aim/robot_test/projection", qos::debug);
    auto predicted_pub = node.details->make_pub<visualization_msgs::msg::MarkerArray>(
        "/rmcs/auto_aim/robot_test/predicted", qos::debug);

    constexpr auto kDt = 0.01;

    auto pixel_eng   = std::mt19937 { 7 }; // NOLINT(cert-msc51-cpp)
    auto pixel_noise = std::normal_distribution<double> { 0.0, 1.5 };

    auto motion    = MockMotion { };
    auto projector = Projector { pixel_eng, pixel_noise };

    auto model = std::unique_ptr<RobotModel> { };

    auto elapsed     = 0.0;
    auto frame_index = 0;

    constexpr auto kArmorNames = std::array { "px", "py", "nx", "ny" };

    while (rclcpp::ok()) {
        elapsed += kDt;

        const auto stamp  = node.details->rclcpp->now();
        const auto center = motion.position(elapsed);
        const auto theta  = motion.rotation_angle(elapsed);

        const auto scene = MockScene { center, theta };

        const auto true_positions = std::array<Eigen::Vector3d, 4> {
            scene.armors[0].translation.make<Eigen::Vector3d>(),
            scene.armors[1].translation.make<Eigen::Vector3d>(),
            scene.armors[2].translation.make<Eigen::Vector3d>(),
            scene.armors[3].translation.make<Eigen::Vector3d>(),
        };
        const auto true_orientations = std::array<Eigen::Quaterniond, 4> {
            scene.armors[0].orientation.make<Eigen::Quaterniond>(),
            scene.armors[1].orientation.make<Eigen::Quaterniond>(),
            scene.armors[2].orientation.make<Eigen::Quaterniond>(),
            scene.armors[3].orientation.make<Eigen::Quaterniond>(),
        };

        auto observed_id = std::size_t { 0 };
        for (std::size_t idx = 1; idx < true_positions.size(); ++idx) {
            if (true_positions[idx].x() >= true_positions[observed_id].x()) continue;
            observed_id = idx;
        }

        // RobotModel 配准验证
        {
            // 构造假观测：full→Armor2d, part→Lightbar2d（选离原点更近的灯条）
            auto fake_armor2ds  = std::vector<Armor2d> { };
            auto fake_lightbars = std::vector<Lightbar2d> { };
            {
                const auto vis    = projector.compute_visibility(scene);
                const auto ros2cv = [](const Point3d& p) -> Point3d {
                    return Point3d { -p.y, -p.z, p.x };
                };

                for (int aid = 0; aid < 4; ++aid) {
                    if (vis[aid][0] == '-') continue;

                    const auto& bar_l = scene.lightbars[aid * 2];
                    const auto& bar_r = scene.lightbars[aid * 2 + 1];

                    const auto tl = reproject_point(ros2cv(bar_l.upper), projector.camera);
                    const auto bl = reproject_point(ros2cv(bar_l.lower), projector.camera);
                    const auto tr = reproject_point(ros2cv(bar_r.upper), projector.camera);
                    const auto br = reproject_point(ros2cv(bar_r.lower), projector.camera);

                    if (vis[aid][0] == 'f') {
                        if (tl && bl && tr && br)
                            fake_armor2ds.push_back(Armor2d {
                                .genre      = DeviceId::INFANTRY_3,
                                .color      = ArmorColor::BLUE,
                                .shape      = ArmorShape::SMALL,
                                .confidence = 1.0,
                                .tl         = tl->make<cv::Point2f>(),
                                .tr         = tr->make<cv::Point2f>(),
                                .br         = br->make<cv::Point2f>(),
                                .bl         = bl->make<cv::Point2f>(),
                            });
                    } else {
                        // 单灯条：选离原点更近的那条
                        const auto dist_l = std::hypot(bar_l.upper.x, bar_l.upper.y, bar_l.upper.z);
                        const auto dist_r = std::hypot(bar_r.upper.x, bar_r.upper.y, bar_r.upper.z);
                        const auto& bar   = dist_l < dist_r ? bar_l : bar_r;
                        const auto up     = dist_l < dist_r ? tl : tr;
                        const auto lo     = dist_l < dist_r ? bl : br;
                        if (up && lo)
                            fake_lightbars.push_back(Lightbar2d {
                                .color = ArmorColor::BLUE,
                                .upper = Point2d { up->make<cv::Point2f>().x,
                                    up->make<cv::Point2f>().y },
                                .lower = Point2d { lo->make<cv::Point2f>().x,
                                    lo->make<cv::Point2f>().y },
                            });
                    }
                }
            }

            if (!model && !fake_armor2ds.empty()) {
                auto rcfg           = RobotModel::Config { };
                rcfg.camera_feature = projector.camera;
                model = std::make_unique<RobotModel>(std::span { fake_armor2ds }, rcfg);
            }

            if (model) {
                model->predict(kDt);
                model->correct(fake_armor2ds, fake_lightbars);

                // 打印真值可见灯条（成对放 []）+ yaw
                std::print("[true] ");
                const auto vis = projector.compute_visibility(scene);
                for (int aid = 0; aid < 4; ++aid) {
                    if (vis[aid][0] == '-') continue;
                    const auto l = aid * 2;
                    const auto r = aid * 2 + 1;
                    if (vis[aid][0] == 'f') std::print("[{} {}] ", l, r);
                    else {
                        const auto& bar_l = scene.lightbars[l];
                        const auto& bar_r = scene.lightbars[r];
                        const auto dist_l = std::hypot(bar_l.upper.x, bar_l.upper.y, bar_l.upper.z);
                        const auto dist_r = std::hypot(bar_r.upper.x, bar_r.upper.y, bar_r.upper.z);
                        std::print("{} ", dist_l < dist_r ? l : r);
                    }
                }
                std::print("yaw=");
                for (int aid = 0; aid < 4; ++aid) {
                    const auto orientation =
                        scene.armors[aid].orientation.make<Eigen::Quaterniond>();
                    const auto position = scene.armors[aid].translation.make<Eigen::Vector3d>();
                    const auto armor_to_center = orientation * Eigen::Vector3d::UnitX();
                    const auto armor_face      = -armor_to_center;
                    const auto armor_to_cam    = (-position).normalized();
                    const auto yaw = std::acos(std::clamp(armor_face.dot(armor_to_cam), -1.0, 1.0));
                    std::print("p{}={:.0f}° ", aid, yaw * 180.0 / std::numbers::pi);
                }
                std::println("");
            }

            if (frame_index > 0 && frame_index % 10 == 0) {
                const auto st = model->state();
                const auto err_center =
                    std::hypot(st.x - center.x(), st.y - center.y(), st.z - center.z());
                const auto err_angle = util::normalize_angle(st.rotation_angle - theta);
                std::println("    [conv] center_err={:.3f} angle_err={:.3f}° rf={:.3f}/{:.3f} "
                             "rl={:.3f}/{:.3f}",
                    err_center, err_angle * 180.0 / std::numbers::pi, st.radius_forward,
                    MockScene::kRadiusForward, st.radius_lateral, MockScene::kRadiusLateral);
            }
        }

        // 投影图像（20 Hz）
        if (frame_index % 5 == 0) {
            auto img = cv::Mat { Projector::kImageHeight, Projector::kImageWidth, CV_8UC3,
                cv::Scalar { 0, 0, 0 } };

            projector.draw(scene, img);

            // tracked 灯条 id 标注（仅文字，不画圈）
            if (model) {
                for (const auto& tr : model->addition().tracked) {
                    const auto pt = cv::Point {
                        static_cast<int>(tr.point.x),
                        static_cast<int>(tr.point.y),
                    };
                    cv::putText(img, std::to_string(tr.lightbar_id), pt + cv::Point { 8, -8 },
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar { 255, 255, 255 }, 1);
                }
            }

            auto msg            = sensor_msgs::msg::Image { };
            msg.header.stamp    = stamp;
            msg.header.frame_id = "camera_optical_link";
            msg.height          = img.rows;
            msg.width           = img.cols;
            msg.encoding        = "bgr8";
            msg.step            = static_cast<uint32_t>(img.cols * 3);
            msg.data.assign(img.data, img.data + img.total() * img.elemSize());
            image_pub->publish(msg);
        }

        // 3D 可视化
        {
            auto m  = MarkerHelper { };
            m.stamp = stamp;

            for (std::size_t idx = 0; idx < true_positions.size(); ++idx) {
                const auto color = idx == observed_id ? cv::Scalar { 0, 255, 0, 255 } // 亮绿
                                                      : cv::Scalar { 0, 255, 255, 64 }; // 暗黄
                m.add_cube("true_armor", true_positions[idx], true_orientations[idx], 0.003, 0.135,
                    0.056, color);
            }

            m.add_sphere("true_center", center, 0.06, cv::Scalar { 255, 0, 0, 255 });

            std::vector<Eigen::Vector3d> circle_pts;
            constexpr auto kSteps = 36;
            for (int step = 0; step <= kSteps; ++step) {
                const auto phi = 2.0 * std::numbers::pi * step / kSteps;
                circle_pts.emplace_back(center.x() + MockScene::kRadiusForward * std::cos(phi),
                    center.y() + MockScene::kRadiusForward * std::sin(phi), center.z());
            }
            m.add_line_strip("true_circle", circle_pts, 0.005, cv::Scalar { 255, 0, 0, 128 });

            marker_pub->publish(m.array);
        }

        // 预测装甲板 3D 可视化
        if (model) {
            auto m               = MarkerHelper { };
            m.stamp              = stamp;
            const auto predicted = model->full();
            for (std::size_t idx = 0; idx < 4; ++idx) {
                const auto pos = predicted[idx].translation.make<Eigen::Vector3d>();
                const auto orn = predicted[idx].orientation.make<Eigen::Quaterniond>();
                m.add_cube("predicted_armor", pos, orn, 0.004, 0.140, 0.060,
                    cv::Scalar { 0, 0, 255, 128 });
            }
            predicted_pub->publish(m.array);
        }

        ++frame_index;
        rclcpp::sleep_for(std::chrono::milliseconds { 10 });
    }

    return 0;
}
