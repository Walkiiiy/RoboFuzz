# RoboFuzz（中文工程说明）

> 本文档是对当前仓库（`/home/walkiiiy/RoboFuzz`）的完整工程级说明，覆盖：
> 1) 整体架构与工作流程；
> 2) 关键模块与接口（函数/类/CLI）；
> 3) 逐文件说明（含脚本、配置、数据与测试）；
> 4) 如何接入新的 fuzz 项目。

## 1. 项目目标与定位

RoboFuzz 是一个面向 ROS 2 / 机器人系统的语义级 fuzz 框架。它不是只看崩溃的二进制 fuzzer，而是“消息变异 + 状态录制 + 领域 oracle 校验 + 反馈引导”的系统测试框架。

当前代码支持/适配的测试方向包括：
- 通用 ROS topic 消息 fuzz
- PX4（SITL，ROS 通道 / MAVLink 通道 / 参数变异）
- TurtleBot3（SITL / HITL）
- MoveIt2（Panda）
- `ros2_control` / `joint_trajectory_controller`
- `slam_toolbox`（上层建图任务）
- `image_proc`（上层感知处理）
- ROSIDL 类型系统测试
- RCL API 跨语言一致性测试
- CLI 与 API 一致性测试
- Nav2 AMCL 场景（仓库内新增了 `--nav2-amcl` 分支与辅助脚本）
- 配置化目标接入（`src/targets/<name>/config.json` + 必需 `plugin.py`）

## 2. 高层架构

核心链路由以下模块构成：

1. `fuzzer.py`（总控）
- 解析参数、初始化日志目录、构建 `RuntimeConfig`
- 启停目标系统（统一通过 `TargetManager`）
- 发现可 fuzz 目标 topic（`inspect_target`）
- 调度变异（`Scheduler`）
- 执行发布与录包（`Executor`）
- 解析 rosbag（`RosbagParser`）
- 调用 oracle（仅 target plugin，无默认回退）
- 调用 `error_recorder.py` 产出结构化错误证据
- 依据反馈更新队列（feedback-driven queue）

2. `scheduler.py`（策略调度）
- 管理 Cycle / Round
- 支持 single/sequence/repeated/idl_check 等 campaign
- 负责选择字段、阶段（deterministic/havoc）并调用 `mutator.py`

3. `mutator.py`（变异引擎）
- 生成随机初始值
- 提供 AFL 风格 bit/byte/arith/interesting 变异
- 支持 int/float/bool/string 等类型
- 支持 ROSIDL 类型空间变异（builtin + array extension）

4. `executor.py`（执行器）
- 每轮执行前后动作（pre/post hooks）
- 发布消息（默认 `ros2 topic pub --once`）
- 启停 rosbag 录制
- 持久化发送队列样本（`logs/.../queue/msg-*`）

5. `checker.py` + `oracles/*`（语义校验）
- 根据测试模式分发到对应 oracle
- 比较状态消息，输出错误列表
- 维护 RCL API checker / Collision checker（PX4）

6. `rosbag_parser.py`
- 解析 rosbag2 sqlite（`.db3`）
- 根据 `/tmp/start_ts` 与 `/tmp/end_ts` 做时间窗过滤

7. `harness.py`
- 把不同目标系统启动方式统一成函数接口
- PX4/TB3/MoveIt/RCL/CLI/ROSIDL 的启动入口在此封装

## 3. 一次 fuzz 执行的端到端流程

1. 启动：`python3 src/fuzzer.py ...`
2. 初始化：日志目录、queue、可选 shm、可选 PX4 bridge
3. 启动目标（`run_target`）
4. 发现目标 topic 与消息类型（`inspect_target`）
5. 对每个目标创建 `Scheduler`、`Executor`、反馈向量
6. 每一轮：
- `Scheduler` 产出变异消息（或消息序列）
- `Executor` 启动 rosbag 录制、发布消息、停止录制
- `RosbagParser` 解析录制状态
- target `plugin.py` 的 `check_oracle` 调 oracle
- 若报错：落盘 `errors/` + 复制 rosbag 到 `rosbags/{frame}`
- 若反馈更“有趣”：把样本压回 queue
7. 达到 `--maxloop` 或手动中断后清理进程与资源

## 4. 运行入口与 CLI 接口（`src/fuzzer.py`）

### 4.1 主入口
- `main(config)`：主流程控制
- `fuzz_msg(fuzzer, fuzz_targets)`：消息 fuzz 主循环
- `inspect_target(fuzzer)`：根据 ROS graph 发现可 fuzz 订阅
- `inspect_secure_target(fuzzer)`：SROS2 场景发现订阅
- `nav2_amcl_adjust_msg(fuzzer, msg)`：Nav2 输入消息补全/修正

### 4.2 `Fuzzer` 类接口
- `init_cov_map()`：覆盖率共享内存初始化（部分模式跳过）
- `init_shm_data()`：数据共享内存（ROSIDL 相关）
- `init_queue()`：初始化种子队列
- `init_px4_bridge()`：初始化 PX4 通道桥
- `init_state_monitor(watchlist_file)`：启动独立状态监控（当前主链路更多依赖 rosbag）
- `run_target(ros_pkg, ros_node, exec_cmd)`：启动目标
- `kill_target()`：停止目标
- `kill_monitor()`：停止监控
- `destroy()`：统一资源清理

### 4.3 CLI 参数（核心）
- 通用：`--method --schedule --seqlen --repeat --interval --maxloop --logdir --watchlist --fuzz-seed --determ-seed --persistent --no-cov`
- 目标选择：`--target --ros-pkg --ros-node --target-node --exec-cmd`
- PX4：`--px4-sitl-ros --px4-sitl-mav --px4-sitl-pgfuzz --px4-flight-mode --px4-mission`
- TB3：`--tb3-sitl --tb3-hitl --tb3-uri`
- SROS2：`--sros2`
- Nav2：`--nav2-amcl`
- RCL：`--test-rcl --rcl-api --rcl-job`
- CLI 一致性：`--test-cli`
- ROSIDL：`--test-rosidl`
- MoveIt：`--test-moveit`

### 4.4 配置化目标模式（推荐）
- `--target <name>` 会从 `src/targets/<name>/config.json` 读取目标配置
- 必须加载 `src/targets/<name>/plugin.py`（需导出 `TargetPlugin` 类）
- 未提供插件会直接报错退出（不再有默认回退）

### 4.5 目标目录约束
- 当前仅支持单一目标目录：`src/targets/`。
- `TargetManager` 只扫描 `<proj_root>/src/targets/*/config.json`。
- 每个 target 必须提供 `plugin.py`，不再允许默认 oracle 回退。

## 5. 核心模块接口清单

### 5.1 `src/config.py`
- `TestMode(Enum)`：模式枚举
- `RuntimeConfig`：运行态配置对象
  - `find_package_metadata()`：通过 `ros2 pkg prefix` 推导包路径、可执行路径、覆盖率路径

### 5.2 `src/scheduler.py`
- `Campaign(Enum)`：`RND_SINGLE/RND_SEQUENCE/RND_REPEATED/IDL_CHECK/...`
- `Scheduler`：
  - `filter_field_list(whitelist, blacklist)`
  - `init_schedule()`
  - `mutate_generic(config)`
  - `mutate_sequence(config)`
  - `mutate_sequence_mav(config)`
  - `mutate_typemsg(config)`（ROSIDL）
  - `mutate_px4_param(config)`（PGFuzz 风格参数变异）
  - `mutate_moveit_goal(config)` / `mutate_moveit_joint(config)`

### 5.3 `src/mutator.py`
- 随机值与边界：`gen_rand_data/get_rand_val/get_bounds/get_default_val`
- 类型空间变异：`mutate_type/random_builtin_type_except/get_primary_type`
- 数值生成：`gen_int_in_range/gen_float_in_range/gen_special_floats`
- 位级工具：`int*_to_bitstr/float*_to_bitstr/str_to_bitstr/bitlist_to_binary`
- 主变异：`mutate_one(dtype, value, stage, pos, arith_val, interesting_idx)`
- 关键常量：`APPLICABLE_STAGES`、`INTERESTING_MAP`、`STAGE_*`

### 5.4 `src/executor.py`
- `ExecMode(Enum)`：`SINGLE/SEQUENCE`
- `Executor`：
  - `prep_execution(msg_type_class, topic_name)`
  - `execute(...)`
  - `start_rosbag(exec_cnt)` / `kill_rosbag()`
  - `start_watching()` / `stop_watching()`（写入 `/tmp/start_ts` 与 `/tmp/end_ts`）
  - `do_ros2_pub(msg)`（默认发布实现）
  - `save_msg_to_queue(msg, frame, subframe)`

### 5.5 `src/checker.py`
- `run_checks(config, msg_list, state_dict, feedback_list)`：oracle 分发
- `run_rpt_checks(config, state_dict_list)`：重复执行一致性检查
- `APIChecker`：解析 `ltrace` 输出并比对跨语言 API 行为
- `CollisionChecker`：监听 Gazebo 接触事件（PX4）

### 5.6 `src/ros_utils.py`
- ROS 接口枚举：`get_all_message_types/get_all_service_types`
- 图查询：`get_publishers/get_subscriptions/get_secure_subscriptions/get_services`
- 类型加载：`get_msg_class_from_name/get_msg_typename_from_class`
- 结构展开：`flatten_nested_dict`

### 5.7 `src/rosbag_parser.py`
- `RosbagParser(db_file)`
- `process_messages()`：按 start/end 时间窗过滤
- `process_all_messages()`：读取所有消息

### 5.8 `src/feedback.py`
- `FeedbackType(Enum)`：`INC/DEC/ZERO/TARGET`
- `Feedback`：`update_value/is_interesting/reset/...`

### 5.9 `src/target_manager.py`
- 自动扫描并注册 `src/targets/*/config.json`
- `get_target_config()` / `apply_target_to_runtime()`
- `start_target()` / `stop_target()` 生命周期托管
- `get_plugin()` 动态加载目标插件
- `get_field_whitelist()` / `get_feedback_defs()` 配置化调度输入

### 5.10 `src/target_plugins.py`
- `BaseTargetPlugin` 基础插件接口
  - `pre_exec_hook(msg)`
  - `post_exec_hook()`
  - `check_oracle(config, msg_list, state_dict, feedback_list)`

## 6. Oracle 模块说明

所有 oracle 统一接口：
- `check(config, msg_list, state_dict_or_pose_list, feedback_list) -> list[str]`

具体文件：
- `oracles/turtlesim.py`：边界、NaN/INF 检查
- `oracles/turtlebot.py`：IMU/Odom/Scan 约束与跨传感器一致性（含 `theta_diff` 反馈）
- `oracles/px4.py`：飞行模式参数约束、传感器一致性、hold 模式位移检测
- `oracles/moveit.py`：关节限位、动作状态、末端位姿偏差、IK 可达性
- `oracles/rosidl.py`：输入/回放消息一致性
- `oracles/nav2_amcl.py`：姿态合法性、协方差、跳变、TF map->odom、诊断级别

## 7. 与目标系统相关的桥接/辅助模块

- `px4_utils.py`
  - `Parameter`（参数变异消息类）
  - `Px4BridgeNode`（ROS/MAVLink 双通道控制）
  - `get_init_trajectory_msg/get_init_manual_control_msg/get_init_parameter_msg`
  - `read_trajectory_seed/read_offboard_mission/conduct_offboard_mission`
- `harness.py`
  - `run_px4_stack_sh/run_tb3_sitl/run_tb3_hitl/run_moveit_harness/run_rcl_api_harness/...`
- `rcl_harness.py`、`cli_harness.py`
  - 接收 fuzzer 发布指令，驱动目标程序并产出 `out-*`、`trace-*`

## 8. 日志与产物约定

每次运行会创建 `logs/<timestamp>/`，并维护 `logs/latest` 软链接。

关键子目录：
- `queue/`：发送消息样本（pickle）
- `metadata/`：每轮元信息与结构化上下文
- `errors/`：原始错误文本 + 结构化错误 JSON
- `cov/`：反馈值落盘
- `rosbags/`：错误用例对应的 rosbag
- `cmd` / `args`：运行命令与参数快照

当前新增的结构化错误记录由 `src/error_recorder.py` 负责，重点文件包括：
- `errors/error-<frame>`：兼容旧流程的原始错误字符串
- `errors/error-<frame>.json`：结构化错误对象，供独立分析器消费
- `metadata/context-<frame>.json`：目标配置、watchlist、plugin 等运行上下文
- `metadata/execution-summary-<frame>.json`：目标是否启动、发布是否成功、rosbag 是否启动等执行链事实
- `metadata/input-summary-<frame>.json`：输入消息摘要，包含 `NaN/INF/极值/时间戳` 标记
- `metadata/observation-summary-<frame>.json`：各 watch topic 的消息计数与类型
- `metadata/diagnostics-summary-<frame>.json`：从 `/diagnostics` 提炼出的级别与错误摘要

这些文件的目的不是在 `src/` 内做“漏洞判断”，而是为仓库外层的 `agent/falseAnalyzer/` 提供稳定证据。

## 8.1 falseAnalyzer：独立错误审查与 LLM 过滤

`falseAnalyzer` 是一个与 fuzz 主流程解耦的独立 triage agent，目录位于：
- `agent/falseAnalyzer/`

设计原则：
- `src/` 负责记录事实，不做价值判断
- `agent/falseAnalyzer/` 负责读取日志、做规则筛选，并可选调用 LLM（当前支持 DeepSeek）
- 先做“是否值得继续探查”的分诊，再决定是否进入 replay / 人工分析 / 安全审查

### 使用方式

先正常运行 RoboFuzz。新版本会在 `src/logs/<run_id>/` 下自动产生结构化错误日志。

规则分诊模式：

```bash
cd /home/walkiiiy/RoboFuzz/agent/falseAnalyzer
python3 -m false_analyzer.cli \
  --run /home/walkiiiy/RoboFuzz/src/logs/<run_id> \
  --output /home/walkiiiy/RoboFuzz/agent/falseAnalyzer/output
```

使用 DeepSeek 做 LLM 分诊：

```bash
export DEEPSEEK_API_KEY="your_key_here"
export DEEPSEEK_BASE_URL="https://api.deepseek.com"
export DEEPSEEK_MODEL="deepseek-chat"

cd /home/walkiiiy/RoboFuzz/agent/falseAnalyzer
python3 -m false_analyzer.cli \
  --run /home/walkiiiy/RoboFuzz/src/logs/<run_id> \
  --output /home/walkiiiy/RoboFuzz/agent/falseAnalyzer/output \
  --use-llm
```

兼容旧日志：
- 历史样本多数仍在 `src/logs/legacy/<run_id>/`
- `falseAnalyzer` 会自动回退读取旧版 `error-*` 文件，但旧样本缺少 `execution/input/observation` 结构化上下文，因此判断置信度通常低于新日志

输出位置：
- `agent/falseAnalyzer/output/<run_id>/summary.json`
- `agent/falseAnalyzer/output/<run_id>/<case_id>.json`

### 现有框架下，什么样的错误更值得探查

经过规则层与 LLM 过滤后，通常更值得继续做 replay 或人工分析的，是下面这些 case：

- 下游输出或状态发生异常，而不是只停留在发布端报错
- 目标自己的 `/diagnostics` 报出 `level >= 2` 或明确内部错误
- 多个信号同时支持异常，例如 `covariance exploded` 加 `diagnostics`，或 `liveness` 加稳定复现
- 出现空间/运动/控制语义破坏，例如 `teleportation`、`tf missing map->odom transform`、MoveIt action 状态错误、末端位姿严重偏差
- 目标仍正常启动且发布成功，但输出 topic 消失、状态估计发散、规划语义失真
- 不是输入本身的直接镜像，而是目标内部处理后放大的异常

在当前框架下，通常优先级较低、应先排除环境因素的错误包括：

- `publish failed` 且栈停在 ROS CLI / type support / import 阶段
- `UnsupportedTypeSupport`、依赖缺失、目标未启动、watchlist 配错
- `watch failed: no messages captured` 但没有更多执行链证据
- 输入本身已经是 `NaN/INF/极端时间戳`，而输出只是同字段机械回显
- 大量重复、无新增后果、无 diagnostics 支撑的弱 oracle 告警

推荐的调查顺序：
1. 先看 `execution-summary`，确认目标启动和发布是否成功
2. 再看 `observation-summary` 与 `diagnostics-summary`，判断异常是否真实发生在目标输出链
3. 最后再结合 `input-summary` 判断它是“系统脆弱性”还是“输入值直接回显”

## 9. 如何接入新的 fuzz 项目（重点）

现在推荐“零改 fuzzer 代码”的配置化接入流程。

### 第 1 步：在 `src/targets/` 下新建目录

例如：
- `src/targets/new_robot/config.json`
- `src/targets/new_robot/plugin.py`（必需）

### 第 2 步：编写 `config.json`

最小模板：

```json
{
  "name": "new_robot",
  "basic": {
    "ros_pkg": "your_pkg",
    "ros_node": "your_node"
  },
  "lifecycle": {
    "managed": true,
    "install_script": "install.sh",
    "verify_cmd": "ros2 pkg prefix your_pkg >/dev/null",
    "skip_ros_pkg_check": false,
    "start_cmd": "ros2 launch your_pkg your.launch.py",
    "stop_cmd": "pkill -f your_pkg",
    "warmup_sec": 5.0
  },
  "fuzzing": {
    "default_topic": "/cmd",
    "default_msg_type": "geometry_msgs/msg/Twist",
    "field_whitelist": [
      ["linear", "x", "float64"],
      ["angular", "z", "float64"]
    ]
  },
  "monitoring": {
    "watchlist": "watchlist.json",
    "plugin": "plugin.py"
  },
  "runtime": {
    "persistent": true
  }
}
```

说明：
- `lifecycle.start_cmd/stop_cmd`：统一由 `TargetManager.start_target/stop_target` 托管
- fuzz 运行不会自动安装依赖；依赖缺失会直接报错退出
- `lifecycle.install_script`：仅用于手工安装（提示命令），不在 fuzz 期间自动执行
- `lifecycle.verify_cmd`：运行前健康检查（例如类型支持、接口可用性）；失败会直接报错退出
- `lifecycle.skip_ros_pkg_check=true`：用于非标准 ROS 包目标，仅依赖 `verify_cmd`
- `runtime`：可把布尔开关注入 `RuntimeConfig`（如 `test_moveit`, `px4_sitl` 等）

### 第 3 步：按需实现 `plugin.py`

插件需导出 `TargetPlugin(BaseTargetPlugin)`，可覆写：
- `pre_exec_hook(msg)`：发布前消息修正
- `post_exec_hook()`：执行后回调
- `check_oracle(...)`：自定义语义校验
- 插件为必需项；缺失会在启动阶段直接报错退出。

### 第 4 步：执行与验证

```bash
cd src
python3 fuzzer.py --target new_robot --method message --schedule single --no-cov
```

检查项：
- `TargetManager` 能识别你的目标（`src/targets/*/config.json`）
- 目标可正常启动/停止
- `watchlist` 录包有效
- oracle 输出可解释错误
- 回归样本可复现（`logs/.../queue` + `rosbags`）

## 10. 逐文件说明（全量）

> 说明规则：按仓库当前文件逐个列出；自动生成/二进制产物也会注明用途。

### 10.1 根目录

- `.gitignore`：忽略构建中间文件、日志、密钥等
- `LICENSE`：MIT 许可证
- `INSTALL.md`：Docker 安装与运行说明（英文）
- `REQUIREMENTS.md`：硬件/软件环境要求（英文）
- `README_legacy.md`：原版英文说明（论文与复现实验说明）
- `hybrid_fuzzing.md`：混合仿真+实机（以 TB3 为例）的操作指南
- `README.md`：本中文工程说明（新）
- `src/targets/`：配置化目标目录（每个目标一个子目录）

### 10.2 `src/` 顶层代码文件

- `fuzzer.py`：主程序入口与总控编排
- `scheduler.py`：变异策略调度器
- `mutator.py`：类型与位级变异引擎
- `executor.py`：执行发布/录包/样本落盘
- `checker.py`：oracle 分发与一致性校验
- `harness.py`：多目标启动桥接
- `config.py`：运行时配置对象与包路径解析
- `constants.py`：全局常量与类型枚举
- `feedback.py`：反馈对象与有趣性判定
- `ros_utils.py`：ROS 类型/图查询工具
- `rosbag_parser.py`：rosbag2 sqlite 解析
- `state_monitor.py`：独立状态监控节点（历史方案）
- `inspector.py`：目标 topic/service 发现工具函数（与 `fuzzer.py` 中实现重叠）
- `tracer.py`：ltrace 解析器
- `rcl_harness.py`：RCL API 一致性 harness
- `cli_harness.py`：CLI/API 一致性 harness
- `px4_utils.py`：PX4 控制与参数变异核心工具
- `px4_ros_bridge.py`：早期 PX4 ROS 桥接测试脚本
- `ros_to_mav.py`：MAVLink 手工控制脚本
- `idltest_replayer.py`：ROSIDL 样本回放工具
- `reproduce_mav.py`：PX4/MAV 重放与复现实验脚本
- `run_robofuzz.sh`：docker 启动封装
- `run_nav2_fuzz_in_container.sh`：Nav2 AMCL 场景一键执行脚本
- `nav2prep.sh`：Nav2 依赖安装脚本（国内镜像）
- `nav2_env_prep.sh`：Nav2 环境准备（apt/source 两路线）
- `nav2build.sh`：Nav2 依赖与源码构建脚本（较重）
- `target_manager.py`：配置化目标扫描/注册/生命周期管理
- `target_plugins.py`：目标插件基础接口
- `qos_override.yaml`：rosbag 录制 QoS 覆写（特别是 `/map` 与 `listen_flag`）
- `requirements.txt`：Python 依赖（`sysv-ipc`、`kinpy`）
- `.dockerignore`：容器构建忽略项
- `out` / `err`：临时输出文件占位（运行时被覆盖）

### 10.3 `src/oracles/`

- `turtlesim.py`：Turtlesim 状态合法性
- `turtlebot.py`：TB3 传感器/运动约束与一致性
- `px4.py`：PX4 飞行约束与一致性
- `moveit.py`：MoveIt2/Panda 运动学与控制语义检查
- `rosidl.py`：ROSIDL 回放一致性
- `nav2_amcl.py`：Nav2 AMCL 稳定性与 TF 语义检查

### 10.4 `src/targets/*/watchlist.json`

- 每个 target 在自身目录维护独立 `watchlist.json`。
- 新接入目标时不再编辑全局 watchlist 目录。

### 10.5 `src/ros2_fuzzer/`（第三方/历史模块）

- `ros_commons.py`：ROS 类型映射与动态加载工具
- `ros_basic_strategies.py`：Hypothesis 策略（string/time/duration/array）
- `ros_fuzzer.py`：基于 Hypothesis 的独立 fuzz CLI
- `process_handling.py`：被测节点存活探测
- `test.py`：示例测试
- `__init__.py`：包标记

### 10.6 `src/coverage/` 与 `src/cov/`

- `coverage/coverage_tool.py`：gcov 报告解析与覆盖率统计
- `coverage/__init__.py`：空
- `coverage/.gitignore`：忽略 `*.gcov`
- `cov/cov-trace.o.c`：sanitizer coverage + shm 写入 hook
- `cov/cov-fs.o.c`：coverage 追踪写文件 hook
- `cov/Makefile`：构建 hook 对象与示例构建命令
- `cov/.gitignore`：忽略 `*.o`

### 10.7 `src/policies/`（SROS2）

- `fuzzer.policy.xml`：fuzzer 策略总入口
- `sros2_node.policy.xml`：sros2 节点策略
- `common/node.xml`：公共 node 策略聚合
- `common/lifecycle_node.xml`：lifecycle node 扩展策略
- `common/node/logging.xml`：`/rosout` 权限
- `common/node/time.xml`：`/clock` 权限
- `common/node/parameters.xml`：参数 topic/service 权限

### 10.8 `src/librcl_apis/`

- `publisher.txt/subscriber.txt/service.txt/client.txt/node.txt/timer.txt/graph.txt/guard_condition.txt/init.txt/time.txt/wait.txt/expand_topic_name.txt/validate_topic_name.txt`
  - 每文件列出一个 RCL API 族，供 `rcl_harness.py` 生成 `ltrace -x ...` 过滤器
- `format.py`：把 API 名单格式化成 `-x func@librcl.so` 的辅助脚本

### 10.9 `src/px4_prep/`

- `parameter_reference.md`：PX4 参数文档（体积大）
- `params.txt`：参数列表原始数据
- `params.pkl`：预处理后的参数元数据（被 `px4_utils.py` 直接读取）
- `preprocess_params.py`：从文档提取参数范围/默认值
- `blacklist.py`：参数黑名单与已测参数名单

### 10.10 `src/missions/`

- `mission_generator.py`：生成 PX4 mission JSON（示例）
- `mission_generator2.py`：生成推力相关 mission
- `mission_generator3.py`：生成 pushdown mission
- `takeoff*.json/church*.json/test.json`：当前仓库为空文件占位（需按场景补充）

### 10.11 `src/utils/`

- `nav2_amcl_feeder.py`：发布假激光与静态 TF，驱动 AMCL
- `replay.py`：历史回放工具（message/service）
- `create_msg.py`：构造 pickle 消息样本
- `msg_viewer.py`：查看 pickle 消息
- `ros-radamsa.sh`：基于 radamsa 的字符串 topic 变异脚本
- `sros_init.sh`：SROS2 keystore/权限初始化
- `speaker.py`：循环向 sros2 topic 发消息
- `clear_shm.sh`：清理共享内存
- `set_focus.sh/reset_focus.sh`：桌面窗口焦点策略辅助脚本

### 10.12 `src/tests/`

- `test_flips_bool.py/test_flips_byte.py/test_flips_int.py/test_flips_float.py`
  - 验证 bit flip 系列变异行为
- `test_arith_int.py/test_arith_float.py`
  - 验证 arithmetic 变异行为
- `test_unicode.py`
  - 验证 Unicode 生成
- `test_kinematics.py`
  - MoveIt/Panda 运动学验证脚本
- `test_tracer.py`
  - `APITracer` 解析验证

### 10.13 其他目录与文件

- `src/states-0.bag/`：示例 rosbag2 数据库文件（`*.db3`, `-shm`, `-wal`）
- `src/logs/`：运行日志目录（运行时生成）

### 10.14 `src/targets/`（配置化目标）

- `target_config.schema.json`：目标配置 schema
- `_shared/install_with_apt.sh`：通用依赖安装与校验 helper
- `README.md`：目标示例索引
- 每个目标目录均包含 `config.json` + `install.sh`（`nav2_amcl` 还包含 `plugin.py`）
- 新目标接入前，建议先阅读 [src/targets/README.md](/home/walkiiiy/RoboFuzz/src/targets/README.md) 中“避免系统性假阳性”一节。
- 接入时至少验证三层是否对齐：
  - 输入层：消息是否真的被目标消费
  - 观测层：watchlist 是否真的看到预期下游信号
  - 判定层：oracle 是否只在有意义异常时报警

## 11. 当前代码状态与后续改进建议

结合代码现状，后续改进可优先做：

1. 结构解耦
- `fuzzer.py` 体量过大（>1800 行），建议拆分：参数解析/目标发现/主循环/模式插件

2. 统一目标插件化
- 把 PX4/TB3/MoveIt/Nav2 的分支逻辑抽象为目标插件接口：
  - `prepare_target()`
  - `build_field_whitelist()`
  - `build_feedbacks()`
  - `pre_exec_hooks()/post_exec_hooks()`
  - `oracle_check()`

3. 异常与资源回收一致性
- 部分路径 `kill_target/kill_rosbag` 仍依赖外部状态；建议上下文管理器化

4. 样本格式统一
- 当前 queue 用 pickle，跨版本可移植性差；建议补充 JSON 序列化层

5. 测试体系
- 现有 `src/tests` 偏脚本验证，建议引入 pytest + CI smoke 场景

6. 文档同步
- `README_legacy.md` 与当前 Nav2 分支能力已有偏差；应以本 README 为主并持续同步

## 12. 常用命令参考

以下命令均基于新接口 `--target`，覆盖当前 `src/targets/` 下所有项目模块示例。

统一行为说明：
- 每个 target 都配置了 `lifecycle.install_script` 和 `lifecycle.verify_cmd`。
- 运行前若依赖未满足，会直接报错退出；请手工执行 `src/targets/<name>/install.sh` 后重试。
- 若安装脚本不存在、执行失败，或安装后 `verify_cmd` 仍失败，程序会直接报错退出，不会进入假运行。

### 12.1 Turtlesim(tested)

```bash
cd src
python3 fuzzer.py --target turtlesim --method message --schedule single --no-cov
```

### 12.2 TurtleBot3 SITL(tested)

```bash
cd src
python3 fuzzer.py --target turtlebot3_sitl --method message --schedule single --no-cov
```

### 12.3 TurtleBot3 HITL

```bash
cd src
python3 fuzzer.py --target turtlebot3_hitl --method message --schedule single --no-cov
```

### 12.4 MoveIt2(tested)

```bash
cd src
python3 fuzzer.py --target moveit2 --method message --schedule single --no-cov
```

### 12.5 Nav2 AMCL(tested)

```bash
cd src
python3 fuzzer.py --target nav2_amcl --method message --schedule single --no-cov
```

### 12.6 PX4 SITL（ROS）

```bash
cd src
python3 fuzzer.py --target px4_sitl_ros --method message --schedule sequence --seqlen 100 --repeat 1 --interval 0.02
```

### 12.7 PX4 SITL（MAVLink）(tested)

```bash
cd src
python3 fuzzer.py --target px4_sitl_mav --method message --schedule sequence --seqlen 100 --repeat 1 --interval 0.1
```

### 12.8 PX4 SITL（PGFuzz 风格参数变异）(tested)

```bash
cd src
python3 fuzzer.py --target px4_sitl_pgfuzz --method message --schedule single --repeat 1 --interval 15
```

### 12.9 ROSIDL 类型系统(tested)

```bash
cd src
python3 fuzzer.py --target rosidl --method message --no-cov
```

### 12.10 RCL API 一致性

```bash
cd src
python3 fuzzer.py --target rcl_api --method message --schedule single --no-cov
```

### 12.11 CLI/API 一致性

```bash
cd src
python3 fuzzer.py --target cli_api --method message --schedule single --no-cov
```

### 12.12 SROS2

```bash
cd src
python3 fuzzer.py --target sros2 --method message --schedule single --no-cov
```

说明：
- `sros2` 运行前需先准备 keystore（可参考 `src/utils/sros_init.sh`）。
- `turtlebot3_hitl` 需先确保实机 SSH 与 `run.sh/kill.sh` 环境可用。
- PX4 模式通常需要额外终端运行 `micrortps_agent -t UDP`。

---

如果你希望，我可以在下一步继续做两件事：
1) 把“新增 fuzz 项目接入”落成一个可直接复制的模板目录（`target_template/` + `oracle_template.py` + `watchlist_template.json`）。
2) 按当前代码再生成一份“接口速查表”（函数签名 + 调用关系图），用于后续重构分工。


## 附录 A：仓库文件全路径索引（逐文件）

> 说明：以下索引覆盖 `rg --files` 的全部文件；每个文件的功能解释见第 10 章对应小节。

- `INSTALL.md`
- `LICENSE`
- `README.md`
- `README_3.13.md`
- `README_legacy.md`
- `REQUIREMENTS.md`
- `Record.md`
- `hybrid_fuzzing.md`
- `src/checker.py`
- `src/cli_harness.py`
- `src/config.py`
- `src/constants.py`
- `src/cov/Makefile`
- `src/cov/cov-fs.o.c`
- `src/cov/cov-trace.o.c`
- `src/coverage/__init__.py`
- `src/coverage/coverage_tool.py`
- `src/err`
- `src/executor.py`
- `src/feedback.py`
- `src/fuzzer.py`
- `src/harness.py`
- `src/idltest_replayer.py`
- `src/inspector.py`
- `src/librcl_apis/client.txt`
- `src/librcl_apis/expand_topic_name.txt`
- `src/librcl_apis/format.py`
- `src/librcl_apis/graph.txt`
- `src/librcl_apis/guard_condition.txt`
- `src/librcl_apis/init.txt`
- `src/librcl_apis/node.txt`
- `src/librcl_apis/publisher.txt`
- `src/librcl_apis/service.txt`
- `src/librcl_apis/subscriber.txt`
- `src/librcl_apis/time.txt`
- `src/librcl_apis/timer.txt`
- `src/librcl_apis/validate_topic_name.txt`
- `src/librcl_apis/wait.txt`
- `src/missions/church-fail.json`
- `src/missions/church-short.json`
- `src/missions/church.json`
- `src/missions/mission_generator.py`
- `src/missions/mission_generator2.py`
- `src/missions/mission_generator3.py`
- `src/missions/takeoff-push.json`
- `src/missions/takeoff-pushdown.json`
- `src/missions/takeoff-quick.json`
- `src/missions/takeoff-quickland.json`
- `src/missions/takeoff.json`
- `src/missions/test.json`
- `src/mutator.py`
- `src/nav2_env_prep.sh`
- `src/nav2build.sh`
- `src/nav2prep.sh`
- `src/oracles/moveit.py`
- `src/oracles/nav2_amcl.py`
- `src/oracles/px4.py`
- `src/oracles/rosidl.py`
- `src/oracles/turtlebot.py`
- `src/oracles/turtlesim.py`
- `src/out`
- `src/policies/common/lifecycle_node.xml`
- `src/policies/common/node.xml`
- `src/policies/common/node/logging.xml`
- `src/policies/common/node/parameters.xml`
- `src/policies/common/node/time.xml`
- `src/policies/fuzzer.policy.xml`
- `src/policies/sros2_node.policy.xml`
- `src/px4_prep/blacklist.py`
- `src/px4_prep/parameter_reference.md`
- `src/px4_prep/params.pkl`
- `src/px4_prep/params.txt`
- `src/px4_prep/preprocess_params.py`
- `src/px4_ros_bridge.py`
- `src/px4_utils.py`
- `src/qos_override.yaml`
- `src/rcl_harness.py`
- `src/reproduce_mav.py`
- `src/requirements.txt`
- `src/ros2_fuzzer/__init__.py`
- `src/ros2_fuzzer/process_handling.py`
- `src/ros2_fuzzer/ros_basic_strategies.py`
- `src/ros2_fuzzer/ros_commons.py`
- `src/ros2_fuzzer/ros_fuzzer.py`
- `src/ros2_fuzzer/test.py`
- `src/ros_to_mav.py`
- `src/ros_utils.py`
- `src/rosbag_parser.py`
- `src/run_nav2_fuzz_in_container.sh`
- `src/run_robofuzz.sh`
- `src/scheduler.py`
- `src/state_monitor.py`
- `src/states-0.bag/states-0.bag_0.db3`
- `src/states-0.bag/states-0.bag_0.db3-shm`
- `src/states-0.bag/states-0.bag_0.db3-wal`
- `src/target_manager.py`
- `src/target_plugins.py`
- `src/tests/test_arith_float.py`
- `src/tests/test_arith_int.py`
- `src/tests/test_flips_bool.py`
- `src/tests/test_flips_byte.py`
- `src/tests/test_flips_float.py`
- `src/tests/test_flips_int.py`
- `src/tests/test_kinematics.py`
- `src/tests/test_tracer.py`
- `src/tests/test_unicode.py`
- `src/tracer.py`
- `src/utils/clear_shm.sh`
- `src/utils/create_msg.py`
- `src/utils/msg_viewer.py`
- `src/utils/nav2_amcl_feeder.py`
- `src/utils/replay.py`
- `src/utils/reset_focus.sh`
- `src/utils/ros-radamsa.sh`
- `src/utils/set_focus.sh`
- `src/utils/speaker.py`
- `src/utils/sros_init.sh`
- `src/targets/README.md`
- `src/targets/cli_api/config.json`
- `src/targets/moveit2/config.json`
- `src/targets/nav2_amcl/config.json`
- `src/targets/nav2_amcl/plugin.py`
- `src/targets/px4_sitl_mav/config.json`
- `src/targets/px4_sitl_pgfuzz/config.json`
- `src/targets/px4_sitl_ros/config.json`
- `src/targets/rcl_api/config.json`
- `src/targets/rosidl/config.json`
- `src/targets/sros2/config.json`
- `src/targets/sros2/watchlist.json`
- `src/targets/target_config.schema.json`
- `src/targets/turtlebot3_hitl/config.json`
- `src/targets/turtlebot3_sitl/config.json`
- `src/targets/turtlesim/config.json`
