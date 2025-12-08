# 海康相机故障速查

### 未找到设备但是 `lsusb` 能找到

一般是用户没有权限导致的，我们需要在**本机**配置 udev 规则，注意不是在 Docker 容器中：

```sh
# 创建 Rules 文件
echo "SUBSYSTEM==\"usb\", ATTR{idVendor}==\"2bdf\", ATTR{idProduct}==\"0001\", MODE=\"0666\"" | sudo tee /etc/udev/rules.d/99-hikcamera.rules
# 重新加载 Rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

执行完上述指令后，理论上就能正确查找相机了