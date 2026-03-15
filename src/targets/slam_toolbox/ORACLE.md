# slam_toolbox Oracle 设计说明

`slam_toolbox` 是一个更上层的建图应用目标。  
它接收 `sensor_msgs/msg/LaserScan`，并在运行中维护地图与 `map -> odom` 相关 TF。

相比完整导航任务链，`slam_toolbox` 更适合当前 RoboFuzz 的原因是：

- 输入路径简单：`/scan`
- 输出语义明确：`/map`、`/tf`
- 不依赖 action server 或复杂任务状态机
- 更容易判断“输入到底有没有真正驱动到目标”

## 1. 设计目标

这条 oracle 主要回答四个问题：

- 输入 scan 发出后，SLAM 是否真的开始产出地图或 TF
- 地图结构是否基本合法
- TF 链是否保持关键完整性，尤其是 `map -> odom`
- 目标是否通过 `/diagnostics` 明确承认内部错误

## 2. 观测面

当前 watchlist 保留三个核心 topic：

- `/map`
- `/tf`
- `/diagnostics`

这样设计是为了先把“建图主链是否正常”看清楚，而不是一开始就录很多外围 topic。

## 3. 判定层次

### 第 0 层：pipeline / liveness

- 若 `/map` 和 `/tf` 都没有任何消息：
  - `pipeline: no slam outputs captured`
- 若只有外围 `/tf` 仍在，但始终没有 `/map`：
  - `pipeline: no /map captured`

这类错误首先说明宿主场景没有被真正驱动，或宿主场景本身没站稳。

### 第 1 层：地图结构合法性

对 `/map` 的 `OccupancyGrid` 检查：

- `resolution` 是否为有限值
- `resolution` 是否落在合理范围
- `width/height` 是否为正
- 地图尺寸是否异常大
- `data` 长度是否等于 `width * height`
- `origin` 是否包含 NaN/INF

这样可以避免“只要看到一张 map 就算正常”的误判。

### 第 2 层：地图有效性

- 若地图始终没有任何已知栅格：
  - `/map never contained any known cells`

这比单次空图更值得关注，因为它意味着：

- SLAM 形式上在线
- 但并没有真正把环境写入地图

### 第 3 层：TF 完整性

对 `/tf` 检查：

- 平移与旋转是否为有限值
- 是否出现 `map -> odom`

这里有一个重要门槛：

- 只有在 `/map` 已经有效出现之后，再去检查 `map -> odom` 是否缺失

这样可以避免把“SLAM 根本没起来，只剩宿主 Gazebo 在发 `/tf`”误判成高频语义错误。

### 第 4 层：诊断信号

若 `/diagnostics` 中任一状态 `level >= 2`：

- `diagnostics error: <name> (<message>)`

这类错误优先级更高，因为它表示目标自己承认内部异常。

## 4. 为什么这样设计

第一版 oracle 刻意没有一上来做很重的地图质量判定，例如：

- 地图与世界的几何重投影误差
- 回环质量
- 全局一致性评分

原因是当前阶段更重要的是：

- 先把宿主场景跑稳
- 先确认输出链是真实可用的
- 先把低噪声、可复现、含义明确的错误筛出来

## 5. 哪些错误更值得深挖

在现有框架下，更值得人工分析的通常是：

- `/map` 结构合法，但长期没有已知栅格
- `/tf` 持续存在，但缺失 `map -> odom`
- `/map` 分辨率、尺寸、数据长度明显异常
- `/tf` 或 `/map origin` 出现 NaN/INF
- diagnostics 明确报告内部错误

## 6. 哪些错误应先降权

- 启动初期的一次性空包
- Gazebo / TurtleBot3 场景刚启动时的短暂无图
- 单次 `/map` 缺失但很快恢复

这类更像宿主场景时序问题，而不是高价值漏洞候选。

## 7. 当前实现边界

当前 plugin 的策略是：

- 保持 `LaserScan` 基本可消费
- 大多数轮次生成稳定、合法的 scan
- 只有低频 fault mode 才做更强语义扰动，如反转或截断 scan

这样做的目的，是让错误更集中到地图和 TF 的真实异常上，而不是让系统每轮都因为输入完全不可消费而报错。
