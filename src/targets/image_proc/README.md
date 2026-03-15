# image_proc Target

这是一个比 Gazebo / Nav2 / SLAM 宿主链更轻的上层感知目标。

宿主场景：

- `image_proc rectify_node`

输入：

- `/camera/image_raw` (`sensor_msgs/msg/Image`)

辅助输入：

- `/camera/camera_info` (`sensor_msgs/msg/CameraInfo`)

观测：

- `/camera/image_rect`
- `/diagnostics`

为什么它适合当前 RoboFuzz：

- 它是上层感知处理节点，不是 ROS 基础中间件
- 不依赖 Gazebo、SLAM、导航 lifecycle 或 action server
- 输入输出清晰，容易判断是否真的被消息驱动
- 更容易做到低噪声 oracle

## 使用方式

先安装依赖：

```bash
bash /home/walkiiiy/RoboFuzz/src/targets/image_proc/install.sh
```

再运行 fuzz：

```bash
cd /home/walkiiiy/RoboFuzz/src
python3 fuzzer.py --target image_proc --method message --schedule single --no-cov
```

启动日志会写到：

- `/home/walkiiiy/RoboFuzz/src/targets/image_proc/log/image_proc.log`

## 当前 oracle 重点

- `/camera/image_rect` 是否真的出现
- 输出图像尺寸、步长、数据长度是否自洽
- 输出编码是否和输入模板一致
- `/diagnostics` 是否报告内部错误

更详细的设计理由见：

- `/home/walkiiiy/RoboFuzz/src/targets/image_proc/ORACLE.md`
