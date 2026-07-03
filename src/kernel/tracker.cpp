#include "tracker.hpp"

#include "module/tracker/model/outpost.hpp"
#include "module/tracker/model/robot.hpp"
#include "utility/logging/printer.hpp"
#include "utility/math/camera.hpp"
#include "utility/serializable.hpp"

#include <algorithm>
#include <ranges>
#include <unordered_map>

using namespace rmcs::kernel;
using namespace rmcs::util;

struct TrackerV2::Impl {
    struct Config : Serializable {
        std::string fallback_color = "RED";
        double timeout_seconds     = 1.5;
        double image_margin        = 20.0;

        static constexpr std::tuple metas {
            // clang-format off
            &Config::fallback_color, "fallback_color",
            &Config::timeout_seconds, "timeout_seconds",
            &Config::image_margin, "image_margin",
            // clang-format on
        };
    } config;

    struct RobotConfig : RobotModel::Config, Serializable {
        static constexpr std::tuple metas {
            // clang-format off
            &RobotConfig::noise_x, "noise_x",
            &RobotConfig::noise_y, "noise_y",
            &RobotConfig::noise_z, "noise_z",
            &RobotConfig::noise_vx, "noise_vx",
            &RobotConfig::noise_vy, "noise_vy",
            &RobotConfig::noise_vz, "noise_vz",
            &RobotConfig::noise_rotation_angle, "noise_rotation_angle",
            &RobotConfig::noise_rotation_speed, "noise_rotation_speed",
            &RobotConfig::noise_observation, "noise_observation",
            // clang-format on
        };
    } robot_config;

    struct OutpostConfig : OutpostModel::Config, Serializable {
        static constexpr std::tuple metas {
            // clang-format off
            &OutpostConfig::process_noise_xy, "process_noise_xy",
            &OutpostConfig::process_noise_z, "process_noise_z",
            &OutpostConfig::process_noise_speed, "process_noise_speed",
            &OutpostConfig::process_noise_angle, "process_noise_angle",
            &OutpostConfig::observation_noise_xy, "observation_noise_xy",
            &OutpostConfig::observation_noise_z, "observation_noise_z",
            &OutpostConfig::observation_noise_yaw, "observation_noise_yaw",
            &OutpostConfig::plate_switch_yaw_min, "plate_switch_yaw_min",
            // clang-format on
        };
    } outpost_config;

    struct {
        Armor2ds armor2ds;
        Armor3ds armor3ds;
        Lightbar2ds lightbars;
    } stored;

    ArmorGenre track_genre = ArmorGenre::UNKNOWN;
    ArmorColor track_color = ArmorColor::DARK;
    DeviceIds track_device = DeviceIds::Full();
    CameraFeature camera;

    std::unordered_map<ArmorGenre, Timestamp> robot_stamps;
    std::unordered_map<ArmorGenre, RobotModel> robot_models;

    Timestamp outpost_stamp;
    std::unique_ptr<OutpostModel> outpost;

    /* @NOTE: 假装这里有一个大符的 Model */

    Printer logging { "TrackerV2" };
    Addition addition;

    explicit Impl(const YAML::Node& yaml) {
        if (auto ret = config.serialize(yaml); !ret) {
            logging.error("TrackerV2 初始化错误: {}", ret.error());
            throw std::runtime_error { "无法构造 TrackerV2" };
        }
        if (auto ret = robot_config.serialize(yaml["robot"]); !ret) {
            logging.error("RobotModel config error: {}", ret.error());
            throw std::runtime_error { "无法构造 RobotModel Config" };
        }
        if (auto ret = outpost_config.serialize(yaml["outpost"]); !ret) {
            logging.error("OutpostModel config error: {}", ret.error());
            throw std::runtime_error { "无法构造 OutpostModel Config" };
        }

        /*^^*/ if (config.fallback_color == "RED") {
            track_color = ArmorColor::RED;
        } else if (config.fallback_color == "BLUE") {
            track_color = ArmorColor::BLUE;
        } else {
            logging.error("invalid color {}", config.fallback_color);
        }
    }

    auto clean() noexcept {
        stored.armor2ds.clear();
        stored.armor3ds.clear();
        stored.lightbars.clear();
    }

    auto store(std::span<const Armor2d> items) {
        const auto kWidth  = camera.camera_matrix[0][2] * 2.0;
        const auto kHeight = camera.camera_matrix[1][2] * 2.0;

        for (const auto& item : items) {
            if (item.color == track_color && track_device.contains(item.genre)) {
                const auto min_x = std::min({ item.tl.x, item.tr.x, item.bl.x, item.br.x });
                const auto max_x = std::max({ item.tl.x, item.tr.x, item.bl.x, item.br.x });
                const auto min_y = std::min({ item.tl.y, item.tr.y, item.bl.y, item.br.y });
                const auto max_y = std::max({ item.tl.y, item.tr.y, item.bl.y, item.br.y });

                if (min_x < config.image_margin || max_x > kWidth - config.image_margin) continue;
                if (min_y < config.image_margin || max_y > kHeight - config.image_margin) continue;

                stored.armor2ds.push_back(item);
            }
        }
    }
    auto store(std::span<const Armor3d> items) {
        for (const auto& item : items) {
            if (item.color == track_color && track_device.contains(item.genre)) {
                stored.armor3ds.push_back(item);
            }
        }
    }
    auto store(std::span<const Lightbar2d> items) {
        for (const auto& item : items) {
            if (item.color == track_color && track_device.contains(item.genre)) {
                stored.lightbars.push_back(item);
            }
        }
    }

    auto execute(Timestamp timestamp) {
        addition.tracked2d.clear();
        addition.tracked3d.clear();
        addition.lightbars.clear();
        addition.infos.clear();

        { // 前哨站观测超时
            const auto dt = std::chrono::duration<double> {
                timestamp - outpost_stamp,
            };
            if (dt.count() > config.timeout_seconds) {
                outpost       = nullptr;
                outpost_stamp = timestamp;
            }
        }
        { // @NOTE: 假装这里有大符的超时处理
        }
        // 机器人观测超时处理
        std::erase_if(robot_stamps, [&](const auto& item) {
            const auto [id, stamp] = item;

            const auto dt = std::chrono::duration<double> { timestamp - stamp };
            if (dt.count() > config.timeout_seconds) {
                robot_models.erase(id);
                return true;
            }
            return false;
        });

        struct Target final {
            std::vector<Armor2d> armor2ds;
            std::vector<Armor3d> armor3ds;
            std::vector<Lightbar2d> bars;
        };
        auto seen = std::unordered_map<ArmorGenre, Target> { };

        for (const auto& armor : stored.armor2ds) {
            seen[armor.genre].armor2ds.push_back(armor);
        }
        for (const auto& armor : stored.armor3ds) {
            seen[armor.genre].armor3ds.push_back(armor);
        }
        for (const auto& bar : stored.lightbars) {
            seen[bar.genre].bars.push_back(bar);
        }

        { // 迭代前哨站 Model
            constexpr auto kId = ArmorGenre::OUTPOST;
            if (seen.contains(kId)) {
                const auto& armors = seen[kId].armor3ds;
                /// @NOTE:
                ///  前哨站为单装甲板输入(120 度，也只能看到一块)，邻侧灯条的信息
                ///  已经在距离优化那个步骤消费掉了，所以这里不需要传入，有时间这
                ///  里也会换成纯 2d 观测，到时候就完全不需要 Armor3d 了
                if (!armors.empty()) {
                    if (outpost == nullptr) {
                        outpost = std::make_unique<OutpostModel>(armors.front());
                        outpost->configure(outpost_config);
                    } else {
                        const auto dt = std::chrono::duration<double> {
                            timestamp - outpost_stamp,
                        };
                        outpost->predict(dt.count());
                        outpost->correct(armors.front());
                    }
                    outpost_stamp = timestamp;
                }
            }
        }
        { // @NOTE: 故作姿态地假装在迭代大符
        }
        // 迭代机器人 Model
        for (const auto& [id, target] : seen) {
            if (id == DeviceId::OUTPOST) {
                continue;
            }
            if (robot_models.contains(id) == false) {
                robot_models.try_emplace(id, robot_config);
                robot_models[id].update_camera(
                    std::bit_cast<std::array<double, 9>>(camera.camera_matrix),
                    camera.distort_coeff);
                robot_models[id].update_transform({
                    .translation = camera.translation,
                    .orientation = camera.orientation,
                });
                if (!robot_models[id].init(target.armor2ds)) {
                    robot_models.erase(id);
                    robot_stamps.erase(id);
                    continue;
                } else {
                    logging.info("Init OK with {}", get_enum_name(id));
                }
            } else {
                const auto dt = std::chrono::duration<double> {
                    timestamp - robot_stamps[id],
                };

                auto& model = robot_models[id];
                model.update_transform({
                    .translation = camera.translation,
                    .orientation = camera.orientation,
                });
                model.predict(dt.count());
                model.correct(target.armor2ds, target.bars);

                if (model.diverged()) {
                    robot_models.erase(id);
                    robot_stamps.erase(id);
                    logging.warn("{} is diverged", get_enum_name(id));
                    continue;
                }
            }
            robot_stamps[id] = timestamp;
        }

        // 选择目标并填充调试信息
        const auto calculate = [&](DeviceId id, const Point3d& p) -> double {
            /// @NOTE:
            ///  占位符实现，按照目标中心到摄像机视角光轴的
            ///  距离比较优先级，后续可能会引入更复杂的判断
            ///  标准，也可能不会（
            const auto distance_score =
                compute_distance2cam_x({ camera.translation, camera.orientation }, p);
            const auto memory_score = (id == track_genre) ? 0 : 1;

            return distance_score * memory_score;
        };

        auto result = Trackable::Unique { };
        auto better = std::numeric_limits<double>::max();
        {
            if (outpost && outpost->converge()) {
                const auto state = outpost->state();
                const auto score = calculate(DeviceId::OUTPOST, state.get_direction());
                if (better > score) {
                    better = score;
                    result = make_trackable(timestamp, state);

                    track_genre = DeviceId::OUTPOST;
                }
                std::ranges::copy(outpost->full(), std::back_inserter(addition.tracked3d));

                const auto a = state.rotation_angle;
                const auto v = state.rotation_speed;
                addition.infos.push_back({
                    .text  = std::format("a: {:+.1f} | v: {:+2.2f}", a, v),
                    .point = Point3d { state.x, state.y, state.z },
                });
            }
            for (const auto& [id, model] : robot_models) {
                if (model.converge()) {
                    const auto state = model.state();
                    const auto score = calculate(id, state.get_direction());
                    if (better > score) {
                        better = score;
                        result = make_trackable(timestamp, state);

                        track_genre = id;
                    }
                    std::ranges::copy( // Armor 2d
                        model.addition().armors, std::back_inserter(addition.tracked2d));
                    std::ranges::copy( // Armor 3d
                        model.full(), std::back_inserter(addition.tracked3d));
                    std::ranges::copy(
                        model.addition().tracked | std::views::transform([](const auto& item) {
                            return Addition::Lightbar { item.lightbar_id, item.point };
                        }),
                        std::back_inserter(addition.lightbars));

                    const auto rv = state.rotation_speed;
                    const auto vx = state.vx;
                    const auto vy = state.vy;
                    addition.infos.push_back({
                        .text  = std::format("rv: {:+2.2f} | v: {:+2.2f}, {:+2.2f}", rv, vx, vy),
                        .point = Point3d { state.x, state.y, state.z },
                    });
                }
            }
        }
        return result;
    }
};

TrackerV2::TrackerV2(const YAML::Node& yaml)
    : pimpl { std::make_unique<Impl>(yaml) } { }

TrackerV2::~TrackerV2() noexcept = default;

auto TrackerV2::update_track_color(CampColor camp) -> void {
    /*^^*/ if (camp == CampColor::RED) {
        pimpl->track_color = ArmorColor::RED;
    } else if (camp == CampColor::BLUE) {
        pimpl->track_color = ArmorColor::BLUE;
    }
    // Do nothing for unknown camp
}
auto TrackerV2::update_track_genre(DeviceIds ids) -> void { pimpl->track_device = ids; }

auto TrackerV2::update_camera(const Transform& t) noexcept -> void {
    pimpl->camera.translation = t.translation;
    pimpl->camera.orientation = t.orientation;
}
auto TrackerV2::update_camera(const std::array<double, 9>& param) noexcept -> void {
    pimpl->camera.from(param);
}
auto TrackerV2::update_camera(const std::array<double, 5>& param) noexcept -> void {
    pimpl->camera.from(param);
}

auto TrackerV2::clean() noexcept -> void { pimpl->clean(); }

auto TrackerV2::store(std::span<const Armor2d> item) -> void { pimpl->store(item); }
auto TrackerV2::store(std::span<const Armor3d> item) -> void { pimpl->store(item); }
auto TrackerV2::store(std::span<const Lightbar2d> item) -> void { pimpl->store(item); }

auto TrackerV2::execute(Timestamp stamp) -> Trackable::Unique { return pimpl->execute(stamp); }

auto TrackerV2::addition() const -> const Addition& { return pimpl->addition; }
