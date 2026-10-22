# 独立相机调参工具 cam_test

`cam_test` 直接使用海康 MVS SDK 采集相机，通过独立网页查看 WebRTC 视频并调整曝光时间和增益。它不依赖原自瞄组件，也不修改原推流、相机库或安装脚本。

## 构建

在 `rmcs_auto_aim_v2` 目录执行：

```sh
cmake -S tool/cxx -B tool/cxx/build-cam-test -DCAM_TEST_ONLY=ON
cmake --build tool/cxx/build-cam-test --target cam_test -j
```

此模式仅构建 `cam_test`，需要本机已安装 MVS SDK、OpenCV（包含 GStreamer 后端）和 GStreamer H.264 编码插件，不需要启动 ROS。推流管线使用 `x264enc`、`rtph264pay`，网页网关复用现有的 `/opt/autoaim/bin/mediamtx`。

## 启动

先启动独立采集程序：

```sh
./tool/cxx/build-cam-test/cam_test
```

另开终端，在同一项目目录启动网页和独立 MediaMTX 网关：

```sh
python3 tool/res/start-cam-test
```

浏览器访问 `http://<机器人IP>:18081/cam_test`，也可直接访问根路径 `/`。网页提供曝光时间（µs）和增益（dB）滑块；曝光滑块上限为 5000 µs，下限及更低的上限受 SDK 范围约束，增益范围由 SDK 提供，显示的实际值仍为硬件回读。控制请求收到设备执行结果后才显示成功。相机断开时禁止调参，程序持续尝试重连，连接恢复后重新读取范围和值。

点击滑块使其获得焦点后，右/上方向键增加数值，左/下方向键减少数值：曝光单次调整 1 µs，增益单次调整 0.02 dB；按住 0.4 秒后加速，曝光每 0.1 秒调整 100 µs，增益每 0.1 秒调整 1 dB，均受滑块范围限制。松键、失去焦点、页面隐藏或相机断连时停止连续调整。

网关脚本从其所在目录读取 `cam-test.html` 和 `mediamtx-cam-test.yml`，不需要复制到 `/opt/autoaim` 或重跑原安装脚本。MediaMTX 不在默认位置时可指定：

```sh
python3 tool/res/start-cam-test --mediamtx /path/to/mediamtx
```

`--port` 可修改网页 HTTP 端口；WebRTC 网关端口仍由独立配置文件指定。

默认相机名称为空，仅在 SDK 恰好枚举到一台相机时自动选择；枚举到多台相机时必须用 `--camera-name` 指定，避免误占原链路的设备。默认曝光时间为 2000 µs，增益为 10 dB，采集帧率为 120 FPS，推流帧率为 60 FPS。可在启动时指定：

```sh
./tool/cxx/build-cam-test/cam_test \
  --camera-name "相机名称" \
  --exposure-us 2000 \
  --gain 10 \
  --framerate 120 \
  --stream-framerate 60
```

网页修改仅对当前运行会话生效，不写入原配置文件；重连会恢复本工具最近成功应用的参数，重新启动则采用命令行参数。采集程序和网页网关分别用各自终端的 `Ctrl+C` 停止；停止网页网关仅回收它启动的 MediaMTX 子进程。

## 与原链路的隔离

| 用途 | 原链路 | cam_test |
| --- | --- | --- |
| 网页/API | TCP 18080 | TCP 18081 |
| WebRTC WHEP | TCP 8889 | TCP 8891 |
| WebRTC ICE | UDP 8189 | UDP 8190 |
| 本机 RTP 输入 | UDP 127.0.0.1:5000 | UDP 127.0.0.1:5001 |
| 流路径 | `autoaim` | `cam_test` |

新网关关闭 RTSP、RTMP、HLS、SRT、控制 API、metrics、pprof 和回放服务，避免第二个 MediaMTX 实例占用原服务的默认端口。远程浏览器需要能访问机器人 TCP 18081、TCP 8891 和 UDP 8190；RTP 输入仅在机器人本机传输。

两套网页和推流服务可以同时运行；同一台实体相机需要由原 capturer 与 `cam_test` 二选一占用。若同时使用两台相机，请通过 `--camera-name` 选择未被原程序占用的设备。

## 本地控制接口

采集程序使用独立目录 `/tmp/autoaim/cam_test/`：

- `commands`：命名管道，每行格式为 `request_id parameter value expires_at_unix_ms\n`，参数仅支持 `exposure_us` 和 `gain`，过期命令不执行。
- `state.json`：原子替换的相机状态，包含进程 PID、连接状态、更新时间、设备回读值及范围、最近命令执行结果。

网页后端提供：

- `GET /api/context`：读取状态。
- `POST /api/camera`：提交 JSON `{"parameter":"exposure_us","value":2000}`；增益使用 `"parameter":"gain"`。

成功响应为 `{"status":"ok","camera":...}`。后端验证有限数值、SDK 范围、状态更新时间（不超过 2 秒）和采集进程存活状态；命令串行执行，最多等待 2 秒获取匹配请求 ID 的设备回读结果。非法请求返回 400，设备拒绝返回 500，断连或状态不可用返回 503，等待超时返回 504；失败响应包含 `message` 和可用时的 `camera` 状态。

本次按要求未执行编译、语法检查、离线测试或启动服务；请在实际相机环境中验证构建、WebRTC 播放、参数回读与断线重连。
