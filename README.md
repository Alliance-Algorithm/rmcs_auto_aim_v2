# 较为现代化的机甲大师自瞄

一些具体的文档和最佳实践可以在 [`doc`](./doc/) 目录中找到，如果需要对该项目进行二次开发，优先查看该目录下的示范和 [`test`](./test/) 中的写法

## 前言

自瞄系统是一个拥有工程细节的项目，在简单概括下，只有接收图像，识别，位姿估计，预测，控制这几步，但自瞄不是一道典型的算法题，它无法通过确定的输入得到确定的结果，我们更多时候需要寻找 BUG 和改良系统，我们当然可以用复用程度低的代码来搭建出一个 Work Around，来应对特定情况下的特定需求，但一旦涉及到应对复杂的比赛环境，多个车辆的定制化需求差异，不同时刻对不同系统的不同数据的可视化或者保存分析需求，低内聚高耦合的代码组织必然带来沉重而悲伤的维护与重构成本

还有一个方面，作为一个典型的工程化落地项目，我们必不可少的会遇到很多算法，重头实现一个已存在的算法是盲目的行为，需要浪费大量时间去组织代码和修正错误，还要进行大量测试，成熟的算法往往可以在 Github 上寻找到测试完备，接口齐全的仓库，我们只需要移植或是拷贝他们，在明确输入输出的情况下良好地使用它们，毕竟我们不是在前沿领域做算法研发

本项目以工程化为最终目的，为机器人提供一个测试与工作流完备，配置友好，重构开销小，错误提示拟人的自瞄系统，方便队员的后续维护和持续开发，为迭代提供舒适的代码基础

## 快速开始

### 项目构建

先确保海康相机的 SDK 正确构建，再保证 `rmcs_executor` 正确构建

```sh
# 进入工作空间的 src/ 目录下
git clone https://github.com/Alliance-Algorithm/ros2-hikcamera.git --depth 1
git clone https://github.com/Alliance-Algorithm/rmcs_auto_aim_v2.git

# 构建依赖
build-rmcs
```

### 运行自瞄

自瞄系统以 Component 的形式集成到 RMCS 控制系统中。在机器人配置文件（如 `sentry.yaml`）的 `components` 列表中添加：

```yaml
- rmcs::AutoAimComponent -> auto_aim_component
```

这是一份可以在本机运行的最小配置:

```yaml
# mock-autoaim.yaml
rmcs_executor:
  ros__parameters:
    update_rate: 1000.0
    components:
      - rmcs::AutoAimComponent -> auto_aim_component

```

你可以用如下方式在本机启动自瞄:

```bash
# 持久化指定运行配置
set-robot mock-autoaim
launch-rmcs

# 或者临时手动指定
ros2 launch rmcs_bringup rmcs.launch.py robot:=mock-autoaim
```

与其他 RMCS 组件不同，自瞄组件的参数不由机器人 YAML 管理，而是由本项目自己的配置文件统一配置，因此实例名可以随意取，不影响参数读取。

配置文件按以下优先级加载（位于运行时 share 目录下），默认只提供 `config.yaml`：

1. 环境变量 `AUTOAIM_CONFIG` 指定的路径
2. `custom.yaml` / `custom.yml`
3. `config.override.yaml` / `config.override.yml`
4. `config.yaml` / `config.yml`

所以我们可以在 `config/` 下面创建一个 `custom.yaml`，作为自己机器人的配置文件，同步至远端时，自瞄会自动选用优先级更高的配置文件，同时除了 `config.yaml` 以外的上述优先级列表中的名字，都写入了 `.gitignore` 中，避免更新仓库时会和本地配置冲突

我们也可以简单用 `export AUTOAIM_CONFIG=unreal.yaml` 来选择默认优先级列表没有的配置文件名，方便本地调试

### 接入自瞄

运行自瞄后，下面的输入输出便处于可用状态，我们需要提供自瞄需要的信息，处理自瞄回传的指令，来实现自瞄控制

#### 输入接口

| 接口名称 | 类型 | 说明 |
|----------|------|------|
| `/auto_aim/camera_transform` | `Eigen::Isometry3d` | 相机在 OdomImu 坐标系下的位姿 |
| `/auto_aim/barrel_direction` | `Eigen::Vector3d` | 枪口在 OdomImu 坐标系下的方向 |
| `/referee/id` | `rmcs_msgs::RobotId` | 机器人 ID，用于判断敌方颜色 |

通过类似下面的方式获取需要的变换：

```cpp
#include <rmcs_description/tf_description.hpp>

// 从 TF 树中查询相机位姿
auto camera_transform = fast_tf::lookup_transform<
    rmcs_description::OdomImu,
    rmcs_description::CameraLink>(*tf);

// 从 TF 树中查询枪口方向
auto barrel_direction = *fast_tf::cast<rmcs_description::OdomImu>(
    rmcs_description::PitchLink::DirectionVector { Eigen::Vector3d::UnitX() }, *tf);
```

#### 输出接口

| 接口名称 | 类型 | 说明 |
|----------|------|------|
| `/auto_aim/should_control` | `bool` | 是否需要云台跟踪 |
| `/auto_aim/should_shoot` | `bool` | 是否可以发弹 |
| `/auto_aim/control_direction` | `Eigen::Vector3d` | 目标方向向量 |
| `/auto_aim/robot_center` | `Eigen::Vector3d` | 目标机器人中心位置 |
| `/auto_aim/yaw_rate` | `double` | yaw 角速度 |
| `/auto_aim/pitch_rate` | `double` | pitch 角速度 |
| `/auto_aim/yaw_acc` | `double` | yaw 角加速度 |
| `/auto_aim/pitch_acc` | `double` | pitch 角加速度 |
| `/auto_aim/feedforward_valid` | `bool` | 前馈数据是否有效 |

## 项目架构

### 文件排布

- `kernel`: 运行时业务内核，与业务逻辑强相关，包含识别、跟踪、位姿估计、火控、可视化等核心流程

- `module`: 特定任务的通用实现模块，不包含运行时逻辑，可在不同上下文中复用

- `utility`: 与业务无关的基础设施数库，包括 `rclcpp` 封装、数学工具、图像处理、进程间共享内存、线程工具等

### 架构设计

本项目采用**单进程多线程**架构，自瞄系统作为 Component 集成到 RMCS 控制系统中：

- **AutoAim**（`auto_aim.hpp`）：在独立的 `worker` 线程中运行自瞄主循环，负责图像采集、目标识别、位姿估计、跟踪、火控解算等算法逻辑

- **AutoAimComponent**（`component.cpp`）：运行在 RMCS 主线程中，负责与 RMCS 控制系统对接

## 调试指南

### Ros2 Topic 可视化

**Foxglove 转发端**

在 Docker 开发容器或者机器人 NUC 容器中下载并启动对应的转发端：

```sh
sudo apt install ros-$ROS_DISTRO-foxglove-bridge
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

在哪里运行程序发布 Topic，就在哪里启动该转发端

**Foxglove 桌面端**

我们可以在浏览器访问 [foxglove网页端](https://app.foxglove.dev/)，或者下载桌面端软件

在 Debian 系中，可以访问该网址 [Download](https://foxglove.dev/download) 下载 Deb 包，或者运行下面指令来安装：

```sh
sudo apt update && sudo apt install foxglove-studio
```

如果使用 ArchLinux，则可以通过 AUR 源来安装：

```sh
paru -S foxglove-bin
```

然后在 `Open Connection` 中打开 `ws://localhost:8765` 这个 `URL`

> 要注意的是，上述 IP 地址取决于运行程序的主机，如果是在机器人上运行的，则需要修改为机器人的 IP，比如：`ws://169.254.233.233:8765`

**确认 Topic 的常见指令**

```sh
# 列举当前正在发布的话题名称
ros2 topic list

# 测量话题的发布频率
ros2 topic hz /topic-name

# 测量话题的发布带宽
ros2 topic bw /topic-name

# 直接输出话题
ros2 topic echo /topic-name
```

**测试**

可以运行自瞄工具下的 `see_outpost` 来测试可视化：

```sh
./build/see_outpost
```

### OPENCV 可视化窗口

如果你使用英伟达 GPU，那你的 OPENCV 的可视化可能会在这一步被拿下，比如`cv::imshow`，设置环境变量永远使用 CPU 渲染即可，另外，其他的 X11 协议的 GUI 程序也可能栽在这里，都可以通过此办法解决

```
# F*** Nvidia
export LIBGL_ALWAYS_SOFTWARE=1
```

### 视频流播放

诚然，依靠 ROS2 的 Topic 来发布 `cv::Mat`，然后使用 `rviz` 或者 `foxglove` 来查看图像不失为一个方便的方法，但经验告诉我们，网络带宽和 ROS2 的性能无法支撑起高帧率高画质的视频显示，所以我们采用 RTP 推流的方式来串流自瞄画面，完全支撑得起 100hz 以上的流畅显示，且在较差的网络环境也能相对流畅地串流，这是 ROS2 不能带给我们的良好体验

使用下面的脚本安装依赖，然后启动即可:

```sh
./tool/install-server.sh local        # 本地安装
./tool/install-server.sh remote       # 远程安装到机器人，需要先设置 remote

start-streamer                        # 启动串流服务，阻塞在当前终端
```

串流服务包含两个部分：

- 推流地址 `rtp://127.0.0.1:5000`（自瞄 RTP 推流目标，需与 config 中 `monitor_port` 一致）
- 网页播放 `http://<ip>:18080/autoaim`（内置一些常用功能）

> 端口可通过环境变量 `AUTOAIM_PLAYER_PORT`（默认 18080）和 `AUTOAIM_MEDIAMTX_PORT`（默认 8889）自定义

详细说明见 [串流文档](./doc/streaming.md)

### 视频流录制

也可通过 FFmpeg 直接录制串流：

```sh
ffmpeg -i "rtp://<ip>:5000" -c:v copy video.mp4
```

## 核心概念

### 0. 依赖隐藏：

C++ 中的一个对象只要内存布局确定，就可以被传递，即便是不完整的类型，著名的 `Impl` 模式就是这样，同样地，我们可以利用这个写法，将一些比较膨胀的依赖隐藏在一个前置声明的类型中，只有当我们真正需要该依赖的上下文时，用过引入完整定义，来使用被隐藏起来的依赖

比如：

```cpp
// object.hpp
struct Object {
    struct Details;
    auto details() -> Details&;
};
// object.details.hpp
struct Object::Details {
    // 一些庞大的上下文，比如 Ros2 和 OpenCV 对象
};
```

在声明接口时，我们可以仅包含 `object.hpp`，作为对象传递，而在 `cpp` 文件中真正需要该上下文以实现功能时，就可以引入 `object.details.hpp`，这样，就实现了依赖的隐藏，头文件细节可以有效收束在实现的编译单元，不会随着头文件的引入而传播：

```cpp
// use_object.hpp
#include "object.hpp"
auto use_object(Object&) -> void;

// use_object.cpp
#include "use_object.hpp"
#include "object.details.hpp"
auto use_object(Object& object) -> void {
    auto& details = object.details();
    // 取出一些惊人而膨胀的上下文，比如：`rclcpp::Node`
}
```

通过这种方式，我们可以有效缩短**增量编译**的时间，特别是用到了诸如 `rclcpp`，`eigen`，`opencv` 等庞然大物时

### 1. 非侵入式：

使用继承与虚函数作为接口是**典型的侵入式多态**，但这并不意味着我们不能使用虚函数，相反，我们不得不用虚函数，它作为 Cpp 主要的运行时多态实现（还有一个是类型擦除），是实现运行时方法和上下文注册所必要的

现代 Cpp 常以 concept 作为接口，比如协程对象的实现，是“组合”理念的体现

对于“组合优于继承”，我们有：

- 实现 concept 约束不需要引入依赖（即接口声明的头文件），即非侵入式，重构开销更小，和依赖隐藏的理念契合
- concept 的组合的开销小于抽象类的组合
- 最小化运行时抽象，使用最少的虚函数来完成运行时注册，面向用户的编译期多态通过模板实现
- concept 可以利用 `static_assert` 等工具来提供更友善的编写期提示

一个典型的例子：[`kernel/capturer.cpp`](https://github.com/Alliance-Algorithm/rmcs_auto_aim_v2/blob/main/src/kernel/capturer.cpp)，它使用了 `Capturer` 的特征，但是一部分是编译期多态接口，一部分是运行时多态接口，编译期接口用于重复性初始化，多态接口用于注册

### 2. 提前编写期检查：

书接上回，在使用 concept 作为接口约束时，应大量使用 `requires`，`static_assert` 等手段来约束，特别是 `static_assert`，它可以将信息用文字传达给开发者，相当友好

### 3. 推迟运行时多态：

我们需要思考，运行时多态是必要的吗？很多时候我们引入了虚函数，多了很多重复性代码，使用基类指针持有对象，但对于真正的运行时，其实并没有那么多接口需求

### 4. 自动化与测试：

一个工程化项目，其测试代码应该占据**一半左右**的代码量，特别是对于 RM 这种对代码稳定性有高要求的场景，同时 `CI/CD` 的妥善使用，可以大大降低我们在更新，部署等场景所花费的精力
