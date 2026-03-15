# Target Examples

本目录提供 RoboFuzz 原有项目的配置化示例。每个子目录至少包含 `config.json`。

- `turtlesim`: 对应原 `--ros-pkg turtlesim --ros-node turtlesim_node`
- `turtlebot3_sitl`: 对应原 `--tb3-sitl`
- `turtlebot3_hitl`: 对应原 `--tb3-hitl`
- `moveit2`: 对应原 `--test-moveit`
- `px4_sitl_ros`: 对应原 `--px4-sitl-ros`
- `px4_sitl_mav`: 对应原 `--px4-sitl-mav`
- `px4_sitl_pgfuzz`: 对应原 `--px4-sitl-pgfuzz`
- `rosidl`: 对应原 `--test-rosidl`
- `rcl_api`: 对应原 `--test-rcl`
- `cli_api`: 对应原 `--test-cli`
- `sros2`: 对应原 `--sros2`
- `nav2_amcl`: 新增配置化示例（含 `plugin.py`）

运行方式（示例）：

```bash
cd src
python3 fuzzer.py --target turtlesim --schedule single --method message --no-cov
```

说明：
- `lifecycle.managed=false` 的目标继续复用现有 `fuzzer.py` 里的 legacy 启停/harness 分支。
- `lifecycle.managed=true`（或设置 `start_cmd`）的目标由 `TargetManager.start_target()` 统一托管。
