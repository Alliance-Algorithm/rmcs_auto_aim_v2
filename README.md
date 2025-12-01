# 较为现代化的机甲大师自瞄

## 前言

自瞄系统是一个拥有工程细节的项目，在简单概括下，只有接收图像，识别，位姿估计，预测，控制这几步，但自瞄不是一道典型的算法题，它无法通过确定的输入得到确定的结果，我们更多时候需要寻找 BUG 和改良系统，我们当然可以用复用程度低的代码来搭建出一个 Work Around，来应对特定情况下的特定需求，但一旦涉及到应对复杂的比赛环境，多个车辆的定制化需求差异，不同时刻对不同系统的不同数据的可视化或者保存分析需求，低内聚高耦合的代码组织必然带来沉重而悲伤的维护与重构成本

还有一个方面，作为一个典型的工程化落地项目，我们必不可少的会遇到很多算法，重头实现一个已存在的算法是盲目的行为，需要浪费大量时间去组织代码和修正错误，还要进行大量测试，成熟的算法往往可以在 Github 上寻找到测试完备，接口齐全的仓库，我们只需要移植或是拷贝他们，在明确输入输出的情况下良好地使用它们，毕竟我们不是在前沿领域做算法研发

本项目以工程化为最终目的，为机器人提供一个测试与工作流完备，配置友好，重构开销小，错误提示拟人的自瞄系统，方便队员的后续维护和持续开发，为开发提供舒适的代码基础

## 核心概念

依赖隐藏：

非侵入式：

自动化与测试：

## 部署步骤

先确保海康相机的 SDK 正确构建，再保证 `rmcs_exetutor` 正确构建，如果要运行 RMCS 控制系统的话

```sh
# 进入工作空间的 src/ 目录下
git clone https://github.com/Alliance-Algorithm/ros2-hikcamera.git --branch 2.0 --depth 1
git clone https://github.com/Alliance-Algorithm/rmcs_auto_aim_v2.git

# 构建依赖
build-rmcs

# 启动运行时
ros2 run rmcs_auto_aim_v2 rmcs_auto_aim_v2_runtime

# 运行示例程序
cd /path/to/rmcs_auto_aim_v2/test/
cmake -B build
cmake --build build -j

# 以 example 开头的程序包含了很多具体业务的单独运行时实现
./build/example_xxx
```

## 项目架构

### 文件排布

- `kernel`: 运行时封装，与业务逻辑强相关，包含业务流程，数据流动，参数配置等

- `module`: 特定特务的通用实现模块，但不包含运行时逻辑

- `utility`: 和运行业务无关的基本数据结果，基本算法和辅助工具，与一些第三方库的有限接口封装

### 任务调度

TODO

## 调试指南

### OPENCV 可视化窗口

如果你使用英伟达 GPU，那你的 OPENCV 的可视化可能会在这一步被拿下，比如`cv::imshow`，设置环境变量永远使用 CPU 渲染即可

```
# F*** Nvidia
export LIBGL_ALWAYS_SOFTWARE=1
```

### 视频流播放

诚然，依靠 ROS2 的 Topic 来发布 `cv::Mat`，然后使用 `rviz` 或者 `foxglove` 来查看图像不失为一个方便的方法，但经验告诉我们，网络带宽和 ROS2 的性能无法支撑起高帧率高画质的视频显示，所以我们采用 RTP 推流的方式来串流自瞄画面，这会带来一些的延迟（大概1s吧），但推流完全支撑得起 100hz 以上的流畅显示，且在较差的网络环境也能相对流畅地串流，这是 ROS2 不能带给我们的良好体验，至于延迟，我想也没有人同时看着枪口和视频画面调参吧，网络较好的情况下，延迟不足 1s

首先打开`config/config.yaml`文件，将 `use_visualization`设置为`true`，然后配置推流参数：

```yaml
visualization:
    # ......
    framerate: 80
    monitor_host: "127.0.0.1"
    monitor_port: "5000"
    # ......
```

需要配置的只有帧率和主机网络地址，帧率需要同`capturer`模块的帧率一致（也许应该自动读取相机的帧率，以后再论吧），`host`填自己电脑的 IPv4 地址（注意是和运行自瞄的主机同一局域网下的地址），端口随意，别和本机服务冲突就行了

然后启动项目：

```sh
ros2 launch rmcs_auto_aim_v2 launch.py
```

如果相机正常连接的话，且推流模块正确加载，则会看到以下日志：

```
[...] [INFO] [...] [Capturer]: Connect to capturer successfully
[...] [INFO] [...] [visualization]: Visualization session is opened
[...] [INFO] [...] [visualization]: Sdp has been written to: /tmp/auto_aim.sdp
```

接下来只需要将 `/tmp/auto_aim.sdp` 文件拷贝到自己电脑上，使用能够打开`SDP`文件的视频播放器打开即可，也可以使用指令：

```sh
# 如果自瞄运行在机器人上，就加上 --remote 参数
play-autoaim --user username [--remote][--no-copy]
```

随后你会看到这样的输出：

```
/workspaces/RMCS/main/RMCS (main*) » play-autoaim --user creeper                                       ubuntu@creeper
creeper@localhost's password: 
auto_aim.sdp                                                                       100%   70   330.8KB/s   00:00    
✅ 文件已拷贝到宿主机：/tmp/auto_aim.sdp
creeper@localhost's password: 
---------------------------------------------------------------------------------------------------------------------
/workspaces/RMCS/main/RMCS (main*) » [0000565203226630] main libvlc: 正在以默认界面运行 vlc。使用“cvlc”可以无界面-
```

电脑便自动打开 VLC 播放视频流了，SDP 文件会经过`机器人 -> 容器 -> 本地`到 `/tmp/auto_aim.sdp/` 目录

脚本默认使用 VLC 作为视频流播放器，默认延迟较高，可以将播放器的播放缓存设置为 0 来获得**低延迟的串流体验**：`Tools -> Preferences -> search 'caching'`

愉快调试吧！
