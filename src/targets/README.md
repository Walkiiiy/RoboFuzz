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

以后新增 target，默认按下面这个顺序做，**不要先凭记忆或文档印象直接写 `watchlist.json`**。

### 第 1 步：先只写 `install.sh`

先把依赖安装和 `verify_cmd` 跑通，再谈 target 接入。

原因：
- 很多“看起来像 topic 结构问题”的错误，本质上是包没装完整、类型支持不齐，或者宿主启动方式与本地环境不一致
- 如果安装都没验证，后面的 `config/watchlist/plugin` 很容易建立在错误假设上

### 第 2 步：在真实运行中的容器/环境里探图

安装完成后，**先手工启动目标**，然后在真实 ROS graph 上查询：

```bash
ros2 node list
ros2 topic list
ros2 node info <node_name>
ros2 topic info <topic_name>
```

必须先确认三件事：
- 真实节点名是什么
- 真实输入 topic 是什么
- 真实输出 topic 是什么

这一点很重要：
- `watchlist` 里的 topic 不能靠“包名、教程、印象、launch 文件猜测”来写
- 必须优先以**当前容器里真实运行起来后的 ROS graph**为准

### 第 3 步：再写 `config.json`、`watchlist.json`、`plugin.py`

只有在第 2 步确认真实图结构后，才去写：

- `config.json`
- `watchlist.json`
- `plugin.py`

其中：
- `config.json` 里的 `basic.ros_node` 应写真实节点名
- `fuzzing.default_topic` 应写真实输入 topic
- `watchlist.json` 应只放真实存在且 oracle 真要看的输出 topic
- `plugin.py` 的发布逻辑也必须按真实输入 topic / 配套 topic 来写

### 第 4 步：本地静态检查

```bash
jq empty src/targets/<name>/config.json
python3 -m py_compile src/targets/<name>/plugin.py
bash -n src/targets/<name>/install.sh
```

### 第 5 步：先手工安装（如有依赖）

```bash
bash src/targets/<name>/install.sh
```

### 第 6 步：运行 fuzz

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

## 9. 接入经验：避免系统性假阳性

`robot_state_publisher` 的接入过程暴露了一类很典型的问题：target 可以正常启动、消息也能发布、rosbag 也能录到数据，但每一轮都稳定报同一种错误。这种情况往往不是“发现了很多漏洞”，而是接入拓扑本身没有对齐。

后续新增 target 时，建议优先按下面顺序自检：

1. 先确认 target 是否真的被输入驱动
- 不要只看“目标已启动”
- 要看输入之后，watchlist 中是否出现了预期的下游 topic
- 如果只录到初始化 topic，而始终没有动态输出，优先怀疑接入方式，而不是 oracle

补充一个非常具体的要求：
- 以后不要先写 `watchlist.json` 再去验证
- 正确顺序是：
  - 先装依赖
  - 先在真实运行中的容器里查 `ros2 node list / node info / topic list`
  - 最后再写 `config/watchlist/plugin`

`image_proc` 的接入过程就是一个反例：
- 如果不先看真实 ROS graph，很容易把 `watchlist` 写成“文档上看起来合理、运行时却没有消息”的幻觉 topic
- 这种错误表面上像 oracle 太敏感，实际上是接入拓扑一开始就写偏了

2. plugin 不要每轮都主动制造强语义破坏
- `pre_exec_hook()` 应该先构造稳定、合法的基线消息
- 强 fault 注入应低频、可复现、可开关
- 否则会出现“每轮都红”，但错误价值很低

3. 不要把无意义 fuzz 值原样送进 target
- 可以把极端字段值当作随机种子或 fault selector
- 但发给目标的消息本身应尽量保持可执行、可消费
- 典型例子：时间戳极值、长度非法、数组维度彻底错乱

4. oracle 要区分“观测噪声”和“真实异常”
- 空消息、初始化残留、静态 topic 重复发布，不应直接当高价值错误
- 更应该关注：
  - 没有有效输出
  - 关键 frame/topic 缺失
  - 数值 NaN/INF
  - 持续失活
  - 结构性不一致

补充一条很重要的经验：
- 若宿主进程本身已经崩溃，不要让 oracle 在后续每一轮重复报告同一种派生错误
- 更合理的处理是：
  - 启动阶段就把宿主崩溃拦下来
  - 或先报“关键输出根本未出现”，而不是直接报更深层的语义缺失

5. 新目标接入时，先追求“低噪声基线跑通”
- 第一阶段先让大多数轮次能正常执行
- 第二阶段再逐步增加更强的 fault mode
- 否则错误列表会被系统性假阳性淹没，falseAnalyzer 也很难做出有意义排序

6. 对“启动期发布/静态输出”型目标，优先考虑 persistent
- 如果目标的重要输出主要发生在启动期，或依赖 transient local / latched 语义
- 每轮重启 target 很容易引入 rosbag 订阅时序竞态
- 这类目标更适合在 config 的 `runtime` 中启用 `persistent: true`
- 否则就会出现“上一轮正常、下一轮空包”的抖动，而这通常不是有价值错误

### `robot_state_publisher` 具体教训

- 只靠 `/tf_static` 不能说明输入真的打到了 target
- 把 `header.stamp` 固定成不合适的值，会让动态 `/tf` 根本不出现
- oracle 若把空 `TFMessage` 直接记错，会形成稳定误报
- 更合理的做法是：
  - 用 fuzz 值作为场景种子
  - 发给 target 的消息保持时间语义合法
  - 先确认有“有效 transform”再做结构性校验

因此，后续接入新模块时，应该把目标拆成三层来验证：

- 输入层：消息是否被目标真正消费
- 观测层：watchlist 是否能看到预期下游信号
- 判定层：oracle 是否只在有意义异常时报警

## 10. 当前内置目标示例

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
- `joint_trajectory_controller`
- `robot_state_publisher`
- `nav2_bt_navigator`
- `slam_toolbox`
- `image_proc`
