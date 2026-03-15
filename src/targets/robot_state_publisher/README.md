# robot_state_publisher Target

这是一个比控制器类目标更适合 RoboFuzz 当前框架的“完整节点型”目标。

默认启动方式不依赖外部 demo，而是使用本目录自带的最小 URDF：

- `params.yaml` 内置两关节机器人
- `start.sh` 用该参数文件启动 `robot_state_publisher`

默认输入：

- `/joint_states`

默认观测：

- `/tf`
- `/tf_static`

为什么这个目标适合：

- 它是独立节点，不依赖 controller manager / action server / fake hardware
- 输入输出清晰，容易判断是否真打到了目标
- oracle 可以同时检查 liveness、数值稳定性、tf 结构完整性、frame 跳变

当前第一版策略：

- 使用 `pre_exec_hook()` 固定 joint 名称与数组维度
- 以 scheduler 变异出的 `header.stamp` 作为场景种子，在合法基线上生成小幅 position / velocity 扰动
- 仅在少数轮次注入更强的语义 fault：
  - duplicate joint name
  - unknown joint name
  - position/velocity 截断
- 使用 `/tf` 和 `/tf_static` 做结构与数值语义校验

相关文件：

- `config.json`
- `watchlist.json`
- `plugin.py`
- `install.sh`
- `start.sh`
- `params.yaml`
