# image_proc Oracle 设计说明

`image_proc` 是典型的 ROS 上层感知处理节点。  
这里接入的是容器内官方 `image_proc.launch.py` 拉起的 `RectifyNode`，它消费原始图像与相机内参，并输出校正后的图像。

## 1. 设计目标

这条 oracle 主要检查：

- 输入图像是否真的驱动出了 `/camera/image_rect`
- 输出图像结构是否自洽
- 输出是否保持与输入模板一致的尺寸和编码语义
- 节点是否通过 `/diagnostics` 承认内部错误

## 2. 为什么它比重宿主目标更适合

相比 `bt_navigator`、`slam_toolbox` 这类需要大型宿主环境的目标，`image_proc` 更稳：

- 不依赖 Gazebo
- 不依赖地图、定位或控制闭环
- 只要节点本身起来，就能直接验证输入输出链

这很适合当前 RoboFuzz 先追求“低噪声、可解释错误”的阶段。

## 3. 观测面

当前 watchlist 只看：

- `/camera/image_rect`
- `/diagnostics`

这是第一版最小但够用的观察面。

## 4. 判定层次

### 第 0 层：pipeline / liveness

- 若没有任何 `/camera/image_rect`：
  - `pipeline: no rectified image captured`

### 第 1 层：输出结构合法性

对输出图像检查：

- `width/height` 是否为正
- `step` 是否足够承载一行像素
- `data` 长度是否等于 `height * step`
- `encoding` 是否为空或不支持

### 第 2 层：输入输出一致性

由于当前 plugin 会构造稳定模板，所以期望：

- 输出 `width/height` 与输入模板一致
- 输出编码与输入模板一致

若这里持续偏离，更像节点内部处理异常，而不是随机噪声。

### 第 3 层：诊断信号

若 `/diagnostics` 里有 `level >= 2`：

- `diagnostics error: <name> (<message>)`

## 5. 当前实现边界

当前 plugin 不直接让 generic mutator 去破坏整个 `Image.data` 数组。  
它采用的策略是：

- 用被变异的元字段作为种子
- 生成稳定、可消费的灰度图模板
- 同步发布匹配的 `CameraInfo`

这样做的目的是避免“图像本身完全非法导致每轮都红”，把错误集中到更有意义的处理链异常上。
