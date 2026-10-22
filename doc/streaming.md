# 串流

## 架构

自瞄运行时通过 GStreamer 将画面以 RTP 协议推送到本地端口，mediamtx 作为 WebRTC 网关将 RTP 流转换为浏览器可消费的 WebRTC 流，`start-streamer` 同时拉起 mediamtx 和一个播放页 HTTP 服务。

```
自瞄运行时 ──RTP──→ mediamtx ──WebRTC(WHEP)──→ 浏览器播放页
               :5000         :8889              :18080
```

## 安装

```sh
./tool/install-server.sh local        # 本地安装
./tool/install-server.sh remote host  # 远程安装到机器人
```

**本地安装**会：

- 下载 mediamtx（若已安装同版本则复用）到 `/opt/autoaim/bin/`
- 安装 `start-streamer` 和资源文件到 `/opt/autoaim/`
- 安装运行时依赖（curl、python3、gstreamer1.0 等）
- 将 `/opt/autoaim/bin` 加入 `PATH`（写入 `~/.zshrc`）

**远程安装**通过 SSH 将 payload 传输到远程主机并执行安装，适用于直接在机器人 NUC 上部署。

## 启动

```sh
start-streamer
```

输出类似：

```
AutoAim stream server started.

Player page:
  http://127.0.0.1:18080/autoaim

Remote page:
  http://<hostname>:18080/autoaim

WHEP endpoint:
  http://127.0.0.1:8889/autoaim/whep
```

## 推流配置

在自瞄配置文件中开启推流：

```yaml
visualization:
  publishable: true
  drawable: true
  enable_stream: true
  framerate: 60
  monitor_host: "127.0.0.1"
  monitor_port: "5000"
  stream_type: "RTP_H264"
```

| 字段 | 说明 |
|---|---|
| `enable_stream` | 是否开启推流 |
| `framerate` | 推流帧率，需与 `capturer` 帧率一致 |
| `monitor_host` | 推流目标主机 |
| `monitor_port` | 推流目标端口，需与 mediamtx 配置中的 RTP 源端口一致 |
| `stream_type` | 推流格式，支持 `RTP_JEPG` 和 `RTP_H264`，串流服务默认使用 `RTP_H264` |

## mediamtx 配置

配置文件位于 `/opt/autoaim/res/mediamtx.yml`：

```yaml
paths:
  autoaim:
    source: udp+rtp://127.0.0.1:5000
```

若修改了推流端口，需同步更新此处。

## 环境变量

| 变量 | 默认值 | 说明 |
|---|---|---|
| `AUTOAIM_PLAYER_PORT` | `18080` | 播放页 HTTP 端口 |
| `AUTOAIM_MEDIAMTX_PORT` | `8889` | mediamtx API 端口 |
| `AUTOAIM_CAMERA_COMPONENT` | `auto_aim_capturer` | 网页调参目标组件实例名，仅支持字母或下划线开头、其后为字母、数字或下划线的简单名称 |

## 播放页

浏览器访问 `http://<ip>:18080/autoaim`。

播放页基于 WebRTC（WHEP 协议）拉流，断线自动重连，提供截帧和录制功能。

WHEP 端点地址从当前页面 hostname 自动推导为 `http://<hostname>:8889/autoaim/whep`。

## 网页相机调参

原播放页可选接入 `rmcs::AutoAimTunableCapturerComponent`，通过 SDK 回读曝光时间、增益及其范围，并在采集线程中执行网页参数更新。新组件沿用原 capturer 的配置和相机帧输出接口，仅支持 `use_hardware_sync: false` 的自由采集模式；原 `rmcs::AutoAimCapturerComponent`、相机库及 C++ streamer 保持原样，机器人硬同步配置不需要随本次接入修改。

最小组件配置示例：

```yaml
rmcs_executor:
  ros__parameters:
    components:
      - rmcs::AutoAimTunableCapturerComponent -> auto_aim_capturer

auto_aim_capturer:
  ros__parameters:
    camera_name: ""
    exposure_us: 2000.0
    gain: 10.0
    framerate: 120.0
    use_hardware_sync: false
    frame_topic: "/gimbal/auto_aim/camera_frame"
```

在现有 executor 配置中替换对应采集组件，并保留其他组件和原有推流设置；同一台相机由一个采集组件占用。同一进程中的采集实例应统一使用原组件或新组件，避免两种组件各自进行 SDK 设备枚举。组件实例名与网页目标默认均为 `auto_aim_capturer`。每个实例的本地控制目录独立：

```text
/tmp/autoaim/camera/<component_name>/commands
/tmp/autoaim/camera/<component_name>/state.json
```

本机录制场景提供独立的 `auto_aim_record_locally_tunable` 启动 profile，使用新调参组件且实例名仍为 `auto_aim_capturer`，可直接配合默认 `start-streamer` 网页使用。原 `auto_aim_record_locally` profile 保留原采集类，不提供网页调参；机器人硬同步 profile 也保持原样。新配置随 `rmcs_bringup` 构建安装，需要重新构建应用组件和 `rmcs_bringup`。

在 RMCS 工作区根目录构建：

```sh
build-rmcs --packages-up-to rmcs_auto_aim_v2 rmcs_bringup
```

在已加载 RMCS/ROS 环境的终端中启动新配置：

```sh
ros2 launch rmcs_bringup rmcs.launch.py robot:=auto_aim_record_locally_tunable
```

另一个终端运行 `start-streamer`，浏览器访问 `http://<ip>:18080/autoaim`。

同时运行多个采集实例时，在启动网页服务前选择目标，例如：

```sh
AUTOAIM_CAMERA_COMPONENT=capturer_main start-streamer
```

组件名仅用于选择上述目录，不接受斜杠、点或路径跳转；名称无效时仅相机接口不可用。网页和视频仍使用原 HTTP 18080、WHEP 8889、RTP 5000 端口。

曝光滑块限制在 SDK 下限至 `min(SDK 上限, 5000 µs)`，无有效范围时禁用；显示的实际值始终是硬件回读，增益范围由 SDK 提供。鼠标拖动实时提交，页面按 100 ms 节流并串行合并待提交值，后端收到匹配请求 ID 的 SDK 执行结果和回读后才确认成功。曝光的 5000 µs 限制仅用于网页滑块，后端 API 和组件初始配置仍按 SDK 范围校验。

点击滑块使其获得焦点后，右/上方向键增加数值，左/下方向键减少数值：曝光单次调整 1 µs，增益单次调整 0.02 dB；按住 0.4 秒后，曝光每 0.1 秒调整 100 µs，增益每 0.1 秒调整 1 dB。松键、失去焦点、页面隐藏或相机断连时停止连续调整，所有调整均受滑块范围限制。

新组件每 100 ms 原子更新状态文件。`GET /api/context` 保持原 `status/context` 结构，在 `context.camera` 中增加相机状态；未加载新组件、断连、进程退出或状态超过 2 秒时，相机状态会标记为 `connected: false` 并给出 `error`，尽可能保留最近的设备数值，录制、播放和系统状态功能继续工作。使用原 capturer 时不会自动获得调参能力。

`POST /api/camera` 接受 `{"parameter":"exposure_us","value":2000}` 或 `{"parameter":"gain","value":10}`，成功返回 `{"status":"ok","camera":...}`。后端校验有限数值与设备范围，通过独立 FIFO 发送带 UUID 和 1.8 秒有效期的命令，串行等待最多 2 秒确认；非法输入返回 400、设备拒绝返回 500、相机不可用返回 503、确认超时返回 504。网页修改对运行会话生效，不写回原配置文件。

部署时在 `rmcs_auto_aim_v2` 目录沿用 `./tool/install-server.sh local` 或 `./tool/install-server.sh remote host` 更新原网页和 `start-streamer`，随后重启服务。应用组件需要重新构建后使用。

## 录制

除播放页内置录制外，也可通过 FFmpeg 直接录制 RTP 串流：

```sh
ffmpeg -i "rtp://<ip>:5000" -c:v copy video.mp4
```

> RTP 流只能被一个消费者读取，播放页（WebRTC）和 FFmpeg 不能同时消费同一个 RTP 流。如需同时播放与录制，可先用 FFmpeg 复制流：`ffmpeg -i "rtp://<ip>:5000" -c:v copy -f mpegts udp://127.0.0.1:1234`

## 模块接口的使用

首先初始化相关上下文：

```cpp
#include "module/debug/visualization/stream_session.hpp"

using namespace rmcs::debug;

// 检查依赖支持
auto check = StreamContext::check_support();
if (!check) std::println("{}", check.error());

// 配置结构体初始化
auto config   = StreamSession::Config {};
config.target = StreamTarget { host, port };    // 推流目标
config.type   = StreamType::RTP_JEPG;           // 推流格式，一般就用 RTP_JEPG
config.format = VideoFormat { w, h, hz };       // 视频配置

// 创建运行时并打开流输出
auto stream_session = StreamSession {};
stream_session.set_notifier([&](auto msg) {
    std::println("[StreamSession] {}", msg);
});
if (auto result = stream_session.open(config); !result) {
    std::println("{}", result.error());
    // shutdown
}

// 获取 sdp 文件的内容，用视频软件此文件打开即可打开推流
if (auto sdp = stream_session.session_description_protocol()) {
    std::println("{}", sdp.value());
} else {
    std::println("{}", sdp.error());
}
```

初始化成功后就可以开始推流：

```cpp
auto mat = cv::Mat{};
if (!stream_session.push_frame(mat)) {
    std::println("Frame was pushed failed");
}
```

## 运行时接口的使用

```cpp
#include "kernel/visualization.hpp"

using namespace rmcs;

// 传入 YAML::Node 配置进初始化
auto visualization  = kernel::Visualization {};
auto result = visualization.initialize(yaml_node);
if (!result.has_value()) {
    // error handle
}

// 传入 rmcs::Image 进行推流
if (visualization.initialized()) {
    visualization.send_image(*image);
}
```
