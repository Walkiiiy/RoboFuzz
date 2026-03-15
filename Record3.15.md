- 统一Fuzz项目接入接口
- # 角色与背景
你是一个拥有丰富重构经验的 Python 软件架构师。当前我们正在维护一个名为 RoboFuzz 的面向 ROS 2 机器人系统的 Fuzzing 框架。
目前该项目存在严重的耦合问题：接入一个新的测试目标（如 PX4, Nav2, MoveIt2）需要在多个文件中修改硬编码逻辑，例如在 `fuzzer.py` 的 `run_target()`, `kill_target()`, `fuzz_msg()` 中添加大量的 `if target == 'xxx'` 分支，还要在 `harness.py` 中写专属启动函数，在 `checker.py` 中注册 Oracle。

# 核心目标
重构当前的框架架构，实现**“插件化、配置化”**的统一接口。
最终效果：当我们需要接入一个新的 Fuzz 项目时，开发者只需要在 `targets/` 目录下新建一个以项目命名的文件夹（例如 `targets/new_robot/`），并在其中放置一个 `config.json`（定义启动命令、监控话题、变异白名单等），以及一个可选的 `oracle.py`（如果需要特定的语义校验逻辑），即可零代码修改 `fuzzer.py` 完成接入。已有项目（如 Nav2_AMCL 或 Turtlesim）需作为重构后的模板示例。

# 架构设计方案要求
请按照以下设计方案帮我生成重构的核心代码：

## 1. 统一的配置标准 (JSON Schema)
设计一个 `target_config.json` 的结构，至少包含以下信息：
- **基本信息**: 目标名称、被测包名 (`ros_pkg`)、节点名 (`ros_node`)。
- **生命周期 (Lifecycle)**: 启动命令/脚本路径 (`exec_cmd`)，停止命令，以及是否需要预热延时。
- **Fuzzing 策略**: 默认输入 topic、类型。
- **变异约束**: 允许变异的字段白名单（替代目前 `fuzzer.py` 中的 `field_whitelist` 列表）。
- **监控与校验**: 指向 watchlist 文件的路径，以及引用的 oracle 模块名。

## 2. 目标管理器 (TargetManager)
编写一个独立模块 `src/target_manager.py`。
- 实现一个注册表机制，启动时自动扫描 `targets/` 目录，解析所有 `config.json` 并注册目标。
- 提供统一的接口，如 `get_target_config(target_name)`, `start_target()`, `stop_target()`，底层自动根据 JSON 中的配置调用 `subprocess` 或相应的 Harness。

## 3. 剥离 `fuzzer.py` 的硬编码
请给出 `src/fuzzer.py` 重构前后的关键代码 Diff（伪代码或关键逻辑对比）。
- 移除 `run_target()` 和 `kill_target()` 中长串的 `if-elif`。
- 移除 `fuzz_msg()` 中硬编码的 `pre_exec_list`, `post_exec_list` 和 `field_whitelist`，改为从 `TargetManager` 获取。

## 4. 提取 Oracle 与 Hook 的统一接口
定义一个基础类 `BaseTargetPlugin`（或接口），允许特定目标在自己的目录下编写 `plugin.py` 来重写以下方法：
- `pre_exec_hook(msg)` (如 Nav2 需要的 `nav2_amcl_adjust_msg` 修正逻辑)
- `post_exec_hook()`
- `check_oracle(config, msg_list, state_dict, feedback_list)`

## 5. 提供一个完整的迁移示例
请以 `turtlesim` 或 `nav2_amcl` 为例，完整展示它被迁移到新架构后，其所属目录 `targets/nav2_amcl/` 下的：
1. `config.json` 内容。
2. `plugin.py` 内容（包含消息补全修正逻辑和 checker 调用）。

请确保代码风格符合 Pythonic 标准，结构清晰，可扩展性强。