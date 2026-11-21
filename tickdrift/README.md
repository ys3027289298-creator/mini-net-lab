# tickdrift

按参考钟修正 tick 间隔：单帧修正幅度受限、累计漂移可记录，与不漂移基线比对。

`make test` 跑自检。

- 每 tick：`err = client - ref`，`want = base - gain*err`，再夹到 `base*(1±max_adj)`。
- 修正幅度超限：计数并在 `log[]` 中保留请求值，实际按上限应用，绝不硬跳。
- `client` 时间严格递增；参考钟非有限值或倒退的样本只计数、不算修正。
- `cum = client - nominal` 为相对基线的累计漂移，`drift_bound()` 给结构化上界。
