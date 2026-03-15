# Target Examples

本目录提供 RoboFuzz 的配置化目标示例。每个目标子目录都包含：
- `config.json`：目标配置
- `install.sh`：依赖安装与校验脚本
- 可选 `plugin.py`：目标专属 hook/oracle

包含目标：
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

运行方式（示例）：

```bash
cd src
python3 fuzzer.py --target turtlesim --schedule single --method message --no-cov
```

依赖处理规则：
- 运行前先执行 `lifecycle.verify_cmd` 健康检查。
- 检查失败时，自动执行 `lifecycle.install_script`。
- 若脚本不存在、执行失败、或安装后仍未通过检查，立即报错退出，不会进入假运行。

目录说明：
- 根目录 `targets/` 是主配置目录，容器启动脚本会优先挂载并加载它。
- `src/targets/` 是兼容镜像目录，建议与根目录保持同步。
