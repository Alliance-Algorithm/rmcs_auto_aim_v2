# 如何推流视频

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