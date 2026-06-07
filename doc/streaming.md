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

## 播放页

浏览器访问 `http://<ip>:18080/autoaim`。

播放页基于 WebRTC（WHEP 协议）拉流，断线自动重连，提供截帧和录制功能。

WHEP 端点地址从当前页面 hostname 自动推导为 `http://<hostname>:8889/autoaim/whep`。

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

// 检查依赖支持，对于 rmcs-runtime 可以忽略这个步骤
// 镜像内已经内置好了相关依赖
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
