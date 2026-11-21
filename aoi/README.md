# aoi

`make test` 跑自检。

## 一致性校验

每帧的生产 AOI 结果（可带容量裁剪）会按 `interval` 定期与全量广播基线
（半径内所有存活实体，不裁剪）逐观察者对比，差异定位到 `frame / observer / entity`。
判定顺序：

- `fail`：`missing` 或 `extra` 非零（漏发实体必须为零）
- `insufficient`：零差异但样本数 < `min_samples`，报告缺口
- `pass`：零差异且样本达标

开销单独计数：`verify_overhead_pct` 是校验扫描占生产扫描的比例，超过
`OVERHEAD_LIMIT_PCT`（25%）输出 `verify_overhead_note` 建议调大 `interval`。
