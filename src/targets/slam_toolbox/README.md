# slam_toolbox Target

这是一个比完整导航任务链更稳的上层目标。

宿主场景：

- `turtlebot3_gazebo turtlebot3_world.launch.py`
- `slam_toolbox online_async_launch.py`

输入：

- `/scan` (`sensor_msgs/msg/LaserScan`)

观测：

- `/map`
- `/tf`
- `/diagnostics`

为什么它适合当前 RoboFuzz：

- 它是应用层建图目标，不是 ROS 内部基础模块
- 语义后果明确：地图是否稳定、TF 是否自洽
- 比 `bt_navigator` 少了 action/lifecycle/任务状态机那一层复杂性

默认采用 `persistent: true`，避免每轮重启带来的时序抖动。

## 使用方式

先安装依赖：

```bash
bash /home/walkiiiy/RoboFuzz/src/targets/slam_toolbox/install.sh
```

再运行 fuzz：

```bash
cd /home/walkiiiy/RoboFuzz/src
python3 fuzzer.py --target slam_toolbox --method message --schedule single --no-cov
```

启动日志会写到：

- `/home/walkiiiy/RoboFuzz/src/targets/slam_toolbox/log/gazebo.log`
- `/home/walkiiiy/RoboFuzz/src/targets/slam_toolbox/log/slam.log`

如果启动阶段失败，优先看这两个日志。

## 当前 oracle 重点

- `/map` 和 `/tf` 是否真的出现
- `/map` 结构是否合理
- 是否存在 `map -> odom`
- `/diagnostics` 是否报告内部错误

更详细的设计理由见：

- `/home/walkiiiy/RoboFuzz/src/targets/slam_toolbox/ORACLE.md`
