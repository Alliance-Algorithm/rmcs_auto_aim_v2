#include "tracker.hpp"

#include "module/tracker/model/outpost.hpp"
#include "module/tracker/model/robot.hpp"
#include "module/tracker/model/rune.hpp"
#include "utility/logging/printer.hpp"
#include "utility/math/camera.hpp"
#include "utility/serializable.hpp"

#include <algorithm>
#include <ranges>
#include <unordered_map>

using namespace rmcs::kernel;
using namespace rmcs::util;

struct Tracker::Impl {
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

    struct RuneConfig : RuneModel::Config, Serializable {
        static constexpr std::tuple metas {
            // clang-format off
            &RuneConfig::noise_x, "noise_x",
            &RuneConfig::noise_y, "noise_y",
            &RuneConfig::noise_z, "noise_z",
            &RuneConfig::noise_rotation_angle, "noise_rotation_angle",
            &RuneConfig::noise_rotation_speed, "noise_rotation_speed",
            &RuneConfig::noise_face_yaw, "noise_face_yaw",
            &RuneConfig::noise_observation, "noise_observation",
            &RuneConfig::gate_threshold, "gate_threshold",
            &RuneConfig::init_seed_mean_error, "init_seed_mean_error",
            &RuneConfig::init_seed_max_error, "init_seed_max_error",
            &RuneConfig::init_center_gate, "init_center_gate",
            &RuneConfig::init_pitch_bound, "init_pitch_bound",
            // clang-format on
        };
    } rune_config;

    struct {
        Armor2ds armor2ds;
        Armor3ds armor3ds;
        Lightbar2ds lightbars;
        std::vector<RuneIcon> icons;
        std::vector<RuneBullseye> bullseyes;
    } stored;

    bool aim_intent  = false;
    bool aim_cleanup = false;

    DeviceIds track_devices = DeviceIds::Full();
    DeviceId track_genre    = DeviceId::UNKNOWN;
    ArmorColor track_color  = ArmorColor::DARK;
    CameraFeature camera;

    std::unordered_map<DeviceId, Timestamp> robot_stamps;
    std::unordered_map<DeviceId, RobotModel> robot_models;

    Timestamp outpost_stamp;
    std::unique_ptr<OutpostModel> outpost;

    Timestamp rune_stamp, rune_corrected_stamp;
    std::unique_ptr<RuneModel> rune;

    Printer logging { "TrackerV2" };
    Addition addition;

    explicit Impl(const YAML::Node& yaml) {
        auto compat_yaml = YAML::Clone(yaml);
        if (!compat_yaml["fallback_color"] && compat_yaml["enemy_color"]) {
            compat_yaml["fallback_color"] = compat_yaml["enemy_color"];
        }

        if (auto ret = config.serialize(compat_yaml); !ret) {
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
        if (const auto rune_node = yaml["rune"]; rune_node && !rune_node.IsNull()) {
            if (auto ret = rune_config.serialize(rune_node); !ret) {
                logging.error("RuneModel config error: {}", ret.error());
                throw std::runtime_error { "无法构造 RuneModel Config" };
            }
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
        stored.icons.clear();
        stored.bullseyes.clear();
    }

    auto store(std::span<const Armor2d> items) {
        const auto kWidth  = camera.camera_matrix[0][2] * 2.0;
        const auto kHeight = camera.camera_matrix[1][2] * 2.0;

        for (const auto& item : items) {
            if (item.color == track_color && track_devices.contains(item.genre)) {
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
            if (item.color == track_color && track_devices.contains(item.genre)) {
                stored.armor3ds.push_back(item);
            }
        }
    }
    auto store(std::span<const Lightbar2d> items) {
        for (const auto& item : items) {
            if (item.color == track_color && track_devices.contains(item.genre)) {
                stored.lightbars.push_back(item);
            }
        }
    }
    auto store(std::span<const RuneIcon> items) {
        if (!track_devices.contains(DeviceId::RUNE)) return;
        std::ranges::copy(items, std::back_inserter(stored.icons));
    }
    auto store(std::span<const RuneBullseye> items) {
        if (!track_devices.contains(DeviceId::RUNE)) return;
        std::ranges::copy(items, std::back_inserter(stored.bullseyes));
    }

    auto execute(Timestamp timestamp) {
        addition.tracked2d.clear();
        addition.tracked3d.clear();
        addition.lightbars.clear();
        addition.rune_features.clear();
        addition.rune_polygon.reset();
        addition.infos.clear();

        { // 前哨站观测超时
            const auto dt = std::chrono::duration<double> {
                timestamp - outpost_stamp,
            };
            if (dt.count() > config.timeout_seconds) {
                outpost       = nullptr;
                outpost_stamp = timestamp;

                if (track_genre == DeviceId::OUTPOST && aim_intent && aim_cleanup) {
                    track_genre = DeviceId::UNKNOWN;
                }
            }
        }
        { // 大符超时处理
            if (rune != nullptr) {
                const auto dt = std::chrono::duration<double> {
                    timestamp - rune_corrected_stamp,
                };
                if (dt.count() > config.timeout_seconds) {
                    rune = nullptr;

                    if (track_genre == DeviceId::RUNE && aim_intent && aim_cleanup) {
                        track_genre = DeviceId::UNKNOWN;
                    }
                }
            }
        }
        // 机器人观测超时处理
        std::erase_if(robot_stamps, [&](const auto& item) {
            const auto [id, stamp] = item;

            const auto dt = std::chrono::duration<double> { timestamp - stamp };
            if (dt.count() > config.timeout_seconds) {
                robot_models.erase(id);

                if (track_genre == id && aim_intent && aim_cleanup) {
                    track_genre = DeviceId::UNKNOWN;
                }
                return true;
            }
            return false;
        });

        struct Target final {
            std::vector<Armor2d> armor2ds;
            std::vector<Armor3d> armor3ds;
            std::vector<Lightbar2d> bars;
        };
        auto seen = std::unordered_map<DeviceId, Target> { };

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
            constexpr auto kId = DeviceId::OUTPOST;
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

                        if (outpost->diverged()) {
                            outpost = nullptr;
                            logging.warn("{} is diverged", get_enum_name(kId));
                        }
                    }
                    outpost_stamp = timestamp;
                }
            }
        }
        { // 迭代大符
            if (!stored.icons.empty() || !stored.bullseyes.empty()) {
                if (rune == nullptr) {
                    auto model = std::make_unique<RuneModel>(rune_config);
                    model->update_camera(std::bit_cast<std::array<double, 9>>(camera.camera_matrix),
                        camera.distort_coeff);
                    model->update_transform({
                        .translation = camera.translation,
                        .orientation = camera.orientation,
                    });
                    if (model->init(stored.icons, stored.bullseyes, timestamp)) {
                        rune                 = std::move(model);
                        rune_stamp           = timestamp;
                        rune_corrected_stamp = timestamp;
                        logging.info("Init OK with {}", get_enum_name(DeviceId::RUNE));
                    }
                } else {
                    const auto dt = std::chrono::duration<double> {
                        timestamp - rune_stamp,
                    };

                    rune->update_transform({
                        .translation = camera.translation,
                        .orientation = camera.orientation,
                    });
                    rune->predict(dt.count(), timestamp);
                    rune_stamp           = timestamp;
                    const auto corrected = rune->correct(stored.icons, stored.bullseyes);

                    if (rune->diverged()) {
                        rune = nullptr;
                        logging.warn("{} is diverged", get_enum_name(DeviceId::RUNE));
                    } else if (corrected) {
                        rune_corrected_stamp = timestamp;
                    }
                }
            }
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
            std::ignore = id;
            /// @NOTE:
            ///  占位符实现，按照目标中心到摄像机视角光轴的
            ///  距离比较优先级，后续可能会引入更复杂的判断
            ///  标准，也可能不会（
            const auto distance_score =
                compute_distance2cam_x({ camera.translation, camera.orientation }, p);
            return distance_score;
        };
        const auto locked = aim_intent && track_genre != DeviceId::UNKNOWN;

        auto result = Trackable::Unique { };
        auto better = std::numeric_limits<double>::max();
        auto device = DeviceId::UNKNOWN;
        {
            if (!locked || DeviceId::OUTPOST == track_genre) {
                if (outpost && outpost->converge()) {
                    const auto state = outpost->state();
                    const auto score = calculate(DeviceId::OUTPOST, state.get_direction());
                    if (better > score) {
                        better = score;
                        result = make_trackable(outpost_stamp, state, DeviceId::OUTPOST);

                        device = DeviceId::OUTPOST;
                    }
                    std::ranges::copy(outpost->full(), std::back_inserter(addition.tracked3d));

                    const auto a = state.rotation_angle;
                    const auto v = state.rotation_speed;
                    addition.infos.push_back({
                        .text  = std::format("a: {:+.1f} | v: {:+2.2f}", a, v),
                        .point = Point3d { state.x, state.y, state.z },
                    });
                }
            }
            if (!locked || DeviceId::RUNE == track_genre) {
                if (rune && rune->converge()) {
                    const auto state = rune->state();
                    const auto score = calculate(DeviceId::RUNE, state.get_direction());
                    if (better > score) {
                        better = score;
                        result = make_trackable(rune_stamp, state, DeviceId::RUNE);

                        device = DeviceId::RUNE;
                    }

                    std::ranges::copy(
                        rune->addition().predicted | std::views::transform([](const auto& item) {
                            return Addition::RuneFeature { item.feature_id, item.point };
                        }),
                        std::back_inserter(addition.rune_features));

                    if (rune->addition().predicted.size() == 6) {
                        auto polygon = Addition::RunePolygon { };
                        auto ok      = true;
                        for (const auto& item : rune->addition().predicted) {
                            if (item.feature_id == 0) {
                                polygon.icon = item.point;
                            } else if (item.feature_id >= 1 && item.feature_id <= 5) {
                                polygon.blades[static_cast<std::size_t>(item.feature_id - 1)] =
                                    item.point;
                            } else {
                                ok = false;
                            }
                        }
                        if (ok) addition.rune_polygon = polygon;
                    }

                    const auto a = state.rotation_angle;
                    const auto v = state.rotation_speed;

                    const auto text_large_rune = [&] {
                        return std::format(
                            "spd_{}(t)={:+.2f}{:+.2f}*sin({:+.2f}{:+.2f}t), e={:.3f}",
                            state.update_count, state.sine_v, state.sine_a,
                            state.sine_phase, state.sine_omega, state.prediction_cost);
                    };
                    const auto text_small_rune = [&] {
                        return std::format(
                            "spd_{}(t)={:+.2f}, e={:.3f}",
                            state.update_count, v, state.prediction_cost);
                    };
                    const auto text_fallback = [&] { return std::format("theta_ekf={:+.2f}", a); };

                    addition.infos.push_back({
                        .text  = state.sine_valid
                            ? text_large_rune()
                            : (state.use_prediction_speed ? text_small_rune() : text_fallback()),
                        .point = Point3d { state.x, state.y, state.z },
                    });
                }
            }
            for (const auto& [id, model] : robot_models) {
                // 锁定时，不回传其他的目标
                if (locked && id != track_genre) {
                    continue;
                }
                if (model.converge()) {
                    const auto state = model.state();
                    const auto score = calculate(id, state.get_direction());
                    if (better > score) {
                        better = score;
                        result = make_trackable(robot_stamps.at(id), state, id);

                        device = id;
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

        /// @NOTE:
        ///  未锁定时更新目标；一旦自瞄意图按下且已有锁定目标，就保持该目标
        ///  最高优先级，即使目标暂时丢失。开启锁定超时清理时，目标超时后会
        ///  解除锁定并允许重新选择
        if (!locked) {
            track_genre = device;
        }
        return result;
    }
};

Tracker::Tracker(const YAML::Node& yaml)
    : pimpl { std::make_unique<Impl>(yaml) } { }

Tracker::~Tracker() noexcept = default;

auto Tracker::update_aim_intent(bool intent) -> void { pimpl->aim_intent = intent; }
auto Tracker::update_aim_cleanup(bool on) -> void { pimpl->aim_cleanup = on; }

auto Tracker::update_track_color(CampColor camp) -> void {
    /*^^*/ if (camp == CampColor::RED) {
        pimpl->track_color = ArmorColor::RED;
    } else if (camp == CampColor::BLUE) {
        pimpl->track_color = ArmorColor::BLUE;
    }
    // Do nothing for unknown camp
}
auto Tracker::update_track_genre(DeviceIds ids) -> void { pimpl->track_devices = ids; }

auto Tracker::update_camera(const Transform& t) noexcept -> void {
    pimpl->camera.translation = t.translation;
    pimpl->camera.orientation = t.orientation;
}
auto Tracker::update_camera(const std::array<double, 9>& param) noexcept -> void {
    pimpl->camera.from(param);
}
auto Tracker::update_camera(const std::array<double, 5>& param) noexcept -> void {
    pimpl->camera.from(param);
}

auto Tracker::clean() noexcept -> void { pimpl->clean(); }

auto Tracker::store(std::span<const Armor2d> item) -> void { pimpl->store(item); }
auto Tracker::store(std::span<const Armor3d> item) -> void { pimpl->store(item); }
auto Tracker::store(std::span<const Lightbar2d> item) -> void { pimpl->store(item); }
auto Tracker::store(std::span<const RuneIcon> item) -> void { pimpl->store(item); }
auto Tracker::store(std::span<const RuneBullseye> item) -> void { pimpl->store(item); }

auto Tracker::execute(Timestamp stamp) -> Trackable::Unique { return pimpl->execute(stamp); }

auto Tracker::addition() const -> const Addition& { return pimpl->addition; }
