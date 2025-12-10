# ROS2 封装的使用

核心目的，是将 rclcpp 的依赖隔离开，这将显著减少单个编译单元的编译时间，减少开发者的`生命开销`

## 日志

```cpp
#include "utility/rclcpp/node.hpp"

using namespace rmcs;

auto rclcpp = util::RclcppNode { "Example" };

std::size_t num = 2333;
rclcpp.info("{}: Hello World!!", num);
rclcpp.warn("{}: Hello World!!", num);
```

## Topic

对于一个发布或者订阅实例，我们采用 PIMPL 将上下文隐藏在实现中，从而保证头文件的清洁，一个标准的实例实现，一般是下面这种形式

```cpp
#include "utility/rclcpp/node.hpp"

struct Armor {
    // Config 结构体是约定的配置上下文
    // 一致的接口设计有助于流畅开发
    struct Config {
        RclcppNode& rclcpp;
        // ...
    };
    explicit Armor(const Config&);
};
```

而在实现中，由于处于不同的编译单元，我们遍可以大胆地引入 rclcpp 的相关依赖：

```cpp
#include "utility/rclcpp/node.details.hpp"

// 从 details 中拿出 rclcpp::Node 等赤裸的上下文
auto& details = config.rclcpp.details;
```

上述封装手段一定程度上增加了封装的复杂度，但对于持续开发和不断增量编译的场景，我认为是利大于弊的

以一个装甲板的可视化为例子：

```cpp
#include "utility/rclcpp/visual/armor.hpp"

using namespace rmcs;
using namespace rmcs::util;
using namespace rmcs::util::visual;

// 构造一个 Node 并设置话题前缀
auto visual = RclcppNode { "example" };
visual.set_pub_topic_prefix("/rmcs/auto_aim/");

// 构造一个 Armor 对象
auto config = Armor::Config{
    .rclcpp = visual,
    // ...
};
auto armor  = Armor{config};

// 然后移动它并更新它，每一次 update 都会触发一次 publish
armor.move(t, q);
armor.update();
```

具体例子可以在 [`test/visualization.cpp`](../test/visualization.cpp) 中查看

构建项目的 `tests` 后可以使用 `./build/example_visualization` 查看效果，打开 `foxglove` 相关应用就可以看到一个不断旋转平移的四块装甲板