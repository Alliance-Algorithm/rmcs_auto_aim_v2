# 较为现代化的机甲大师自瞄

## 项目架构

## 调试指南

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

需要配置的只有帧率和主机网络地址，帧率需要同`capturer`模块的帧率一致（也许应该自动读取相机的帧率，以后再论吧），`host`填自己电脑的 IPv4 地址，端口随意，别和本机服务冲突就行了

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
play-autoaim username localhost
```

随后你会看到这样的输出：

```
/workspaces/RMCS/main/RMCS (main*) » play-autoaim creeper localhost                                   ubuntu@creeper
ssh: connect to host 127.0.0.1 port 2022: Connection refused
scp: Connection closed
⚠️ 从 remote 拷贝失败。是否继续？(y/n)
y
继续执行后续操作…
creeper@localhost's password: 
auto_aim.sdp                                                                       100%   70   330.8KB/s   00:00    
✅ 文件已拷贝到宿主机：/tmp/auto_aim.sdp
creeper@localhost's password: 
---------------------------------------------------------------------------------------------------------------------
/workspaces/RMCS/main/RMCS (main*) » [0000565203226630] main libvlc: 正在以默认界面运行 vlc。使用“cvlc”可以无界面-
```

电脑便自动打开 VLC 播放视频流了，SDP 文件会经过`机器人 -> 容器 -> 本地`到 `/tmp/auto_aim.sdp/` 目录

愉快调试吧！