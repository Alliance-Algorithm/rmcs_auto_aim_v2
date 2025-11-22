# Utility 工具库文档

本文档描述了 `utility` 工具库中各个模块的功能和用途。

## 设计理念

本工具库中的许多组件是为了**隐藏第三方库的头文件**而设计的包装器。第三方库（如 ROS2 的 rclcpp、OpenCV、Eigen、PCL 等）的头文件会增加编译成本，影响编译体验。通过使用包装器提供有限确定的接口，可以：

- **减少编译时间**：避免在头文件中包含大型第三方库头文件
- **降低编译依赖**：使用 PIMPL 模式将实现细节隐藏在 `.cpp` 文件中
- **保持接口稳定**：提供简洁、稳定的接口，减少对第三方库内部变化的依赖
- **改善编译体验**：只暴露必要的接口，减少不必要的类型暴露

因此，建议优先使用这些包装器而不是直接包含原始第三方库头文件。

## 核心工具

### ASCII 艺术
- **[acsii_art.hpp](../src/utility/acsii_art.hpp)**: 提供 ASCII 艺术横幅，用于程序启动时显示 RMCS 标识。

### 线性代数
- **[linear.hpp](../src/utility/math/linear.hpp)**: 提供 `Translation` 和 `Orientation` 结构体，用于表示三维空间中的平移和旋转。支持从符合特定 trait 的对象进行转换，并提供 `copy_to` 方法将数据复制到目标对象。

### 错误处理
- **[panic.hpp](../src/utility/panic.hpp)** / **[panic.cpp](../src/utility/panic.cpp)**: 提供 `panic` 函数，用于程序异常终止。会输出详细的错误信息，包括消息、文件位置、函数名、行号、线程 ID、时间戳和堆栈跟踪。

### 设计模式
- **[pimpl.hpp](../src/utility/pimpl.hpp)**: 提供 PIMPL（Pointer to Implementation）模式的宏定义 `RMCS_PIMPL_DEFINITION`，用于隐藏实现细节。
- **[details.hpp](../src/utility/details.hpp)**: 提供 Details 模式的宏定义 `RMCS_DETAILS_DEFINITION`，用于分离接口和实现细节。

### 序列化
- **[serializable.hpp](../src/utility/serializable.hpp)**: 提供序列化框架，支持从 YAML 节点或 rclcpp 节点读取参数。使用 `Serializable` 结构体和 `MemberMeta` 来定义可序列化的成员，支持类型安全的参数读取。

### 次数限制
- **[times_limit.hpp](../src/utility/times_limit.hpp)**: 提供 `TimesLimit` 类，用于限制某个操作的执行次数。支持启用/禁用、重置计数等功能。

### 图像处理（OpenCV 包装器）
- **[image.hpp](../src/utility/image.hpp)** / **[image.cpp](../src/utility/image.cpp)**: 提供 `Image` 类，封装图像数据和时间戳。使用 PIMPL 模式隐藏 OpenCV 头文件，避免在头文件中暴露 `cv::Mat` 等 OpenCV 类型。
- **[image.details.hpp](../src/utility/image.details.hpp)**: 定义 `Image::Details` 结构体，包含 OpenCV 的 `cv::Mat` 和相关的访问方法。此文件包含 OpenCV 头文件，应仅在实现文件中使用。
- **[image/painter.hpp](../src/utility/image/painter.hpp)**: 图像绘制工具（具体实现需查看源码）。

### ROS2 节点（rclcpp 包装器）
- **[node.hpp](../src/utility/node.hpp)**: 提供 `Node` 类，继承自 `rclcpp::Node`，扩展了日志功能，支持使用 `std::format` 进行格式化输出。提供 `info`、`warn`、`error` 等方法。**注意**：此文件直接继承 rclcpp::Node，如需完全隐藏 rclcpp 头文件，请使用 `rclcpp/node.hpp` 中的 `RclcppNode`。

### 鸭子类型
- **[duck_type.hpp](../src/utility/duck_type.hpp)**: 提供 `duck_array` 模板类，实现鸭子类型检查。支持在编译时验证类型是否满足特定接口要求，并提供类型安全的元素访问。

## 子模块

### 协程 (coroutine)
- **[coroutine/channel.hpp](../src/utility/coroutine/channel.hpp)**: 提供协程通道 `Channel`，用于协程之间的数据传递。支持发送和接收操作，当通道为空时自动挂起等待。
- **[coroutine/common.hpp](../src/utility/coroutine/common.hpp)**: 提供协程任务 `task` 模板，用于管理协程的生命周期。支持立即启动、延迟销毁，并能处理异常。
- **[coroutine/context.hpp](../src/utility/coroutine/context.hpp)**: 定义协程上下文类（当前为空实现）。

### 数学工具 (math)
- **[math/sigmoid.hpp](../src/utility/math/sigmoid.hpp)**: 提供 sigmoid 函数实现，用于数值稳定计算。
- **[math/solve_armors.hpp](../src/utility/math/solve_armors.hpp)**: 提供装甲板正解和逆解算法。`ArmorsForwardSolution` 用于根据机器人位姿计算四个装甲板的位置和姿态。
- **[math/solve_armors.cpp](../src/utility/math/solve_armors.cpp)**: 正解算法的实现。

### ROS2 扩展 (rclcpp 包装器)
- **[rclcpp/node.hpp](../src/utility/rclcpp/node.hpp)**: 提供 `RclcppNode` 类，封装 ROS2 节点的基本功能，包括日志、发布主题前缀管理等。**使用 PIMPL 模式完全隐藏 rclcpp 头文件**，推荐在需要避免编译依赖时使用。
- **[rclcpp/node.cpp](../src/utility/rclcpp/node.cpp)**: `RclcppNode` 的实现，包含所有 rclcpp 相关的头文件。
- **[rclcpp/node.details.hpp](../src/utility/rclcpp/node.details.hpp)**: `RclcppNode::Details` 的定义，包含 rclcpp 类型，应仅在实现文件中使用。
- **[rclcpp/parameters.hpp](../src/utility/rclcpp/parameters.hpp)**: 提供参数接口 `IParams` 和参数管理类 `Parameters`，用于统一参数访问接口。**隐藏 rclcpp 参数访问的细节**。
- **[rclcpp/parameters.cpp](../src/utility/rclcpp/parameters.cpp)**: `Parameters` 的实现，包含 rclcpp 头文件。
- **[rclcpp/rclcpp_param.hpp](../src/utility/rclcpp/rclcpp_param.hpp)**: 提供 `make_params` 函数，用于从 rclcpp 节点创建参数接口实现。**此文件包含 rclcpp 头文件**，应在实现文件中使用。
- **[rclcpp/configuration.hpp](../src/utility/rclcpp/configuration.hpp)**: 提供 `configuration` 函数，用于从 YAML 文件加载配置。**此文件包含 YAML-CPP 头文件**。
- **[rclcpp/visual/armor.hpp](../src/utility/rclcpp/visual/armor.hpp)** / **[rclcpp/visual/armor.cpp](../src/utility/rclcpp/visual/armor.cpp)**: ROS2 可视化消息相关（装甲板可视化），封装 ROS2 可视化消息类型。

### 机器人相关 (robot)
- **[robot/armor.hpp](../src/utility/robot/armor.hpp)**: 定义装甲板相关类型，包括 `ArmorColor`（颜色枚举）、`ArmorShape`（形状枚举）、`ArmorType`（类型）、`LightStrip`（灯条）和 `Armor` 结构体。
- **[robot/color.hpp](../src/utility/robot/color.hpp)**: 定义 `CampColor` 枚举，表示阵营颜色（未知、红色、蓝色）。
- **[robot/id.hpp](../src/utility/robot/id.hpp)**: 定义 `DeviceId` 枚举和 `DeviceIds` 类，用于标识不同类型的机器人设备（英雄、工程、步兵、哨兵等）。提供设备 ID 的位操作和查询功能。

### 日志 (logging)
- **[logging/printer.hpp](../src/utility/logging/printer.hpp)**: 提供 `Printer` 类，用于日志输出。支持不同级别的日志（INFO、WARN、ERROR），使用 `std::format` 进行格式化。
- **[logging/printer.cpp](../src/utility/logging/printer.cpp)**: `Printer` 的实现。

### 共享内存 (shared)
- **[shared/context.hpp](../src/utility/shared/context.hpp)**: 定义共享上下文结构体，包含时间戳和字节数组，用于进程间数据共享。
- **[shared/interprocess.hpp](../src/utility/shared/interprocess.hpp)**: 提供进程间通信的共享内存客户端。`Client` 模板类包含 `Send` 和 `Recv` 两个类，分别用于发送和接收数据。使用版本号机制确保数据一致性。

### 单例模式 (singleton)
- **[singleton/running.hpp](../src/utility/singleton/running.hpp)**: 提供运行状态管理的全局函数 `get_running` 和 `set_running`。
- **[singleton/running.cpp](../src/utility/singleton/running.cpp)**: 运行状态管理的实现。

### 线程工具 (thread)
- **[thread/spsc_queue.hpp](../src/utility/thread/spsc_queue.hpp)**: 提供单生产者单消费者（SPSC）无锁队列的别名定义，基于 Boost.Lockfree 库。
- **[thread/workers.hpp](../src/utility/thread/workers.hpp)**: 提供 `WorkersContext` 类，用于管理工作线程池。支持提交任务并返回 future，任务必须是 noexcept 可调用的。
- **[thread/workers.cpp](../src/utility/thread/workers.cpp)**: `WorkersContext` 的实现。

### 模型 (model)
- **[model/armor_detection.hpp](../src/utility/model/armor_detection.hpp)**: 定义 `ArmorDetection` 结构体，用于表示检测到的装甲板信息。包含角点坐标、置信度、颜色信息和角色信息。支持从原始数据直接反序列化，并提供边界框计算和角点缩放功能。**注意**：此文件包含 OpenCV 头文件（`opencv2/core/types.hpp`），如需完全隐藏 OpenCV 依赖，建议使用包装器模式。

