# falseAnalyzer

`falseAnalyzer` 是 RoboFuzz 的独立错误审查 agent。它不参与 fuzz 主循环，只读取 `src/logs/` 中已有的错误证据，回答一个更重要的问题：

- 这个错误是不是值得进一步探查？

输出产物：

- 每个 case 一份 triage JSON
- 每个 run 一份 summary JSON

## 输入来源

新日志优先读取这些结构化文件：

- `errors/error-<frame>.json`
- `metadata/context-<frame>.json`
- `metadata/execution-summary-<frame>.json`
- `metadata/input-summary-<frame>.json`
- `metadata/observation-summary-<frame>.json`
- `metadata/diagnostics-summary-<frame>.json`

兼容历史日志：

- 如果 run 目录里只有旧版 `errors/error-*`，也可以读取
- 但旧日志缺少执行链和输入摘要，判断会更保守

## 使用方法

### 1. 仅规则层分诊

```bash
cd /home/walkiiiy/RoboFuzz/agent/falseAnalyzer
python3 -m false_analyzer.cli \
  --run /home/walkiiiy/RoboFuzz/src/logs/<run_id> \
  --output /home/walkiiiy/RoboFuzz/agent/falseAnalyzer/output
```

### 2. 使用 DeepSeek 做 LLM 分诊

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

也可以直接传参覆盖：

```bash
python3 -m false_analyzer.cli \
  --run /home/walkiiiy/RoboFuzz/src/logs/<run_id> \
  --output /home/walkiiiy/RoboFuzz/agent/falseAnalyzer/output \
  --use-llm \
  --api-key "$DEEPSEEK_API_KEY" \
  --base-url "https://api.deepseek.com" \
  --model "deepseek-chat"
```

## 输出说明

输出目录：

- `agent/falseAnalyzer/output/<run_id>/summary.json`
- `agent/falseAnalyzer/output/<run_id>/<case_id>.json`

当前重点字段：

- `verdict`
  - `discard`
  - `low_value`
  - `replay_first`
  - `investigate`
  - `high_priority`
- `vuln_level`
  - `V0` 基础设施/非漏洞
  - `V1` 低价值异常
  - `V2` 值得重放的 robustness 问题
  - `V3` 高价值广义漏洞候选
- `investigation_score`
- `likely_root_cause_class`
- `primary_evidence`
- `counter_evidence`
- `llm_used`
- `llm_summary`

## 什么样的错误更值得探查

### 高价值候选

通常满足以下一条或多条：

- 发布成功、目标正常启动，但下游状态或输出 topic 出现异常
- `/diagnostics` 中出现 `level >= 2`
- `covariance exploded`、`teleportation`、`tf missing map->odom transform`
- MoveIt action 状态、motion plan 请求数、目标位姿出现语义性错误
- 不是输入字段的直接回显，而是系统处理后出现了更严重后果
- 多个证据同时支持，例如 diagnostics + 数值发散 + liveness

### 低优先级或先排环境

通常先不要当漏洞挖：

- `publish failed` 且报错停留在 ROS CLI / type support / import
- `UnsupportedTypeSupport`
- target 未启动、依赖未满足、watchlist 配错
- `watch failed: no messages captured` 但缺少更多执行链证据
- 输入本身就有 `NaN/INF/极端值`，输出只是简单镜像
- 大量重复、无新后果、无 diagnostics 的弱告警

## 当前实现边界

当前版本已经支持：

- 规则归类
- 读取新旧两套日志格式
- 可选调用 DeepSeek
- 将模型失败降级为规则结果，不中断整次分析

当前版本还没有做：

- 自动 replay
- 多次重现后再升级优先级
- 基于 target 专属 prompt 的更细粒度策略
