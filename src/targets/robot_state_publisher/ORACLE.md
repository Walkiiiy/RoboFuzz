# robot_state_publisher Oracle 设计说明

`robot_state_publisher` 接收 `sensor_msgs/msg/JointState`，根据 URDF 计算并发布 TF 树。

它是一个很好的 RoboFuzz 目标，因为：

- 输入清晰：`/joint_states`
- 输出清晰：`/tf`、`/tf_static`
- 语义明确：关节状态应映射为稳定、有限、结构自洽的 transform 集合

## 1. 设计目标

oracle 需要回答：

- 输入打到目标后，TF 是否真的发布了
- 发布出的 transform 是否数值稳定
- frame 结构是否完整
- 关节扰动是否导致不合理的大跳变

## 2. 观测面

最小 watchlist：

- `/tf`
- `/tf_static`

理由：

- `/tf_static` 代表固定链路是否正常初始化
- `/tf` 代表动态关节传播是否正常

## 3. 判定层次

### 第 0 层：liveness

- 若 `/tf` 和 `/tf_static` 都没有消息：
  - `pipeline: neither /tf nor /tf_static captured`

### 第 1 层：数值安全

对每个 transform 检查：

- translation / rotation 是否 NaN/INF
- quaternion norm 是否为 0 或明显偏离 1
- translation 是否离谱地大

### 第 2 层：结构完整性

- `frame_id` 不能为空
- `child_frame_id` 不能为空
- 不能出现 `parent == child`
- 不能缺失关键 child frame

### 第 3 层：动态一致性

- 同一 child frame 的连续 transform 不应出现离谱大跳变

这类错误更接近高价值鲁棒性问题，因为它表明：

- 节点内部传播逻辑被异常输入带偏
- 输出不再维持基本的 TF 语义

## 4. 为什么值得 fuzz

相比依赖复杂宿主系统的控制器类目标，`robot_state_publisher` 更适合当前 RoboFuzz：

- 不需要外部 demo
- 不需要 action client
- 不需要 controller manager
- 更容易确认“录到了就是录到了，没录到就是没打中”

## 5. 当前实现边界

当前 plugin 会先把 `JointState` 模板化成固定维度，但不会再每轮都随机破坏语义。

现在的策略是：

- 通用 mutator 仍然变异 `header.stamp`
- plugin 把该变异结果当作“场景种子”
- 大多数轮次只生成小幅、合法的关节扰动
- 只有少数轮次触发更强的 fault mode：
  - duplicate joint name
  - unknown joint name
  - 截断 position/velocity 数组

这样做的目的，是让基线运行尽量稳定，减少“每轮都红”的低价值噪声，把错误集中到更值得看的 TF 结构失真和输出缺失上。
