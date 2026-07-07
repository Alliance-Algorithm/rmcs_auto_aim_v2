## SOP

1. 项目构建方法：

如果该项目位于 RMCS 的工作区下，可以用如下方法构建
```zsh
build-rmcs --packages-up-to rmcs_auto_aim_v2
```

如果位于 `test` 或者 `tool/cxx` 下，则使用下面指令独立构建

```zsh
cmake -B build
cmake --build build -j
```


2. 语法检查

修改代码后，优先进行 Lsp 层面的检查，而不是直接编译：

```zsh
clangd --check=<修改的文件路径>
```
```zsh
clang-tidy -p <构建目录> <修改的文件路径>
```
只有在 clang-tidy 检查通过后，才进行编译验证。


3. 规范

- 项目头文件遵循最小引入原则，如果某个类型通过头文件间接引入了，那就不需要再引入该头文件
- 不应该忽视 warnings


4. 可视化 (`draw_later`)

- API 位置：`src/kernel/visualization.hpp`
- 可绘制类型定义在 `src/utility/image/drawable.hpp`：
  - `Canvas::Text { content, top_left, color }`
  - `Canvas::Area { rect, color }`
  - `Canvas::Point { origin, radius, color }` → 填充圆
  - `Canvas::ArmorShape { shape, color }`
  - 原生类型：`Armor2d`、`Lightbar2d`、`cv::Rect2i`
- 颜色常量位于 `namespace rmcs` 下，BGRA 格式：
  - `kYellow = { 0, 255, 255, 255 }`
  - `kRed`、`kGreen`、`kWhite` 等
- 绘制生命周期：
  - `visual.draw_later(...)` 仅将绘制对象入队
  - 实际绘制发生在 `visual.update_image(image)` 调用时
  - 通常在每帧末尾的 `std::experimental::scope_exit` 中统一调用 `update_image`
- 通用示例：
  ```cpp
  visual.draw_later(Canvas::Point {
      .origin = cv::Point2i { 100, 200 },
      .radius = 5,
      .color  = kYellow,
  });
  ```
- 参考文件：
  - `src/kernel/visualization.cpp`（实现）
  - `src/utility/image/drawable.cpp`（各类型的实际绘制）
