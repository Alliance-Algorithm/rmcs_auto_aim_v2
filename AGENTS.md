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


2. 规范

- 项目头文件遵循最小引入原则，如果某个类型通过头文件间接引入了，那就不需要再引入该头文件
