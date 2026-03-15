# Targets 接入指南

本文档专门说明 RoboFuzz 在 `src/targets/` 下如何接入新 fuzz 对象。

## 1. 目录约束

- 唯一目标目录：`src/targets/`
- 每个目标一个子目录：`src/targets/<target_name>/`
- 运行时只识别该目录，不再扫描其他位置。

## 2. 每个目标必须包含的文件

以 `new_robot` 为例：

- `src/targets/new_robot/config.json`
- `src/targets/new_robot/plugin.py`
- `src/targets/new_robot/watchlist.json`
- `src/targets/new_robot/install.sh`

说明：
- `config.json`：目标声明、生命周期、fuzz 策略
- `plugin.py`：pre/post hook + oracle
- `watchlist.json`：录包监控 topic 列表
- `install.sh`：手工安装脚本（fuzz 期间不会自动执行）

## 3. config.json 字段说明

最小模板：

```json
{
  "name": "new_robot",
  "basic": {
    "ros_pkg": "your_pkg",
    "ros_node": "your_node"
  },
  "lifecycle": {
    "install_script": "install.sh",
    "verify_cmd": "ros2 pkg prefix your_pkg >/dev/null",
    "skip_ros_pkg_check": false,
    "start_cmd": "ros2 launch your_pkg your.launch.py",
    "stop_cmd": "pkill -f 'your_pkg' || true",
    "warmup_sec": 3.0
  },
  "fuzzing": {
    "default_topic": "/input_topic",
    "default_msg_type": "sensor_msgs/msg/Imu",
    "field_whitelist": [
      ["orientation", "x", "float64"],
      ["orientation", "y", "float64"],
      ["orientation", "z", "float64"],
      ["orientation", "w", "float64"]
    ]
  },
  "monitoring": {
    "watchlist": "watchlist.json",
    "plugin": "plugin.py"
  },
  "runtime": {
    "persistent": false
  }
}
```

关键字段：
- `name`：`--target` 使用的名字
- `basic.ros_pkg`：目标 ROS 包名（用于依赖校验）
- `basic.ros_node`：目标节点名（用于识别/日志）
- `lifecycle.start_cmd/stop_cmd`：统一启动/停止命令（必填）
- `lifecycle.verify_cmd`：依赖/健康检查命令（建议必填）
- `lifecycle.install_script`：安装脚本路径（仅手工执行）
- `fuzzing.default_topic/default_msg_type`：默认输入 topic 与消息类型
- `fuzzing.field_whitelist`：允许变异字段
- `monitoring.watchlist`：必须指向本目录 `watchlist.json`
- `monitoring.plugin`：必须是 `plugin.py`

## 4. watchlist.json 怎么写

格式为 `topic -> message_type` 的 JSON 对象：

```json
{
  "/odometry/filtered": "nav_msgs/msg/Odometry",
  "/diagnostics": "diagnostic_msgs/msg/DiagnosticArray"
}
```

规则：
- 只放 oracle 需要的 topic，避免录包过大
- 类型字符串必须与 ROS2 实际类型一致
- topic 名保持绝对路径（建议以 `/` 开头）

## 5. plugin.py 怎么写

`plugin.py` 必须导出类 `TargetPlugin(BaseTargetPlugin)`。

最小模板：

```python
from target_plugins import BaseTargetPlugin


class TargetPlugin(BaseTargetPlugin):
    def pre_exec_hook(self, msg):
        return msg

    def post_exec_hook(self):
        return None

    def check_oracle(self, config, msg_list, state_dict, feedback_list):
        errs = []
        return errs
```

建议：
- `pre_exec_hook`：只做“不会让消息非法崩掉发布器”的修正
- `check_oracle`：只关心可解释的语义错误，返回 `list[str]`
- 对 `state_dict` 访问要容错：topic 不存在时给出明确错误文本

## 6. install.sh 规范

推荐结构：

```bash
#!/usr/bin/env bash
set -euo pipefail

# 1) 检查 ros2/环境
# 2) apt 或源码安装依赖
# 3) 执行 ldconfig（如需要）
# 4) 最后用 verify_cmd 同等标准复检
```

重要：
- fuzz 主流程不会自动执行安装
- 依赖缺失时会报错退出，并提示你手工执行 `install.sh`

## 7. 新目标接入流程（一步一步）

1. 新建目录：`src/targets/<name>/`
2. 复制模板并完成：`config.json`、`plugin.py`、`watchlist.json`、`install.sh`
3. 本地检查：

```bash
jq empty src/targets/<name>/config.json
python3 -m py_compile src/targets/<name>/plugin.py
bash -n src/targets/<name>/install.sh
```

4. 先手工安装（如有依赖）：

```bash
bash src/targets/<name>/install.sh
```

5. 运行 fuzz：

```bash
cd src
python3 fuzzer.py --target <name> --method message --schedule single --no-cov
```

## 8. 常见报错与定位

- `target 'xxx' not registered`：目录名/`name` 字段不一致，或缺 `config.json`
- `must provide monitoring.plugin`：缺 `plugin.py` 或 config 没配 `monitoring.plugin`
- `dependencies are not satisfied`：`verify_cmd` 失败，先手工执行 `install.sh`
- `watch failed: no messages captured`：watchlist topic 配置错误，或目标没发布数据
- `publish failed`：消息类型支持缺失或 topic 类型不匹配

## 9. 当前内置目标示例

- `turtlesim`
- `turtlebot3_sitl`
- `turtlebot3_hitl`
- `moveit2`
- `nav2_amcl`
- `px4_sitl_ros`
- `px4_sitl_mav`
- `px4_sitl_pgfuzz`
- `rosidl`
- `rcl_api`
- `cli_api`
- `sros2`
- `robot_localization`
