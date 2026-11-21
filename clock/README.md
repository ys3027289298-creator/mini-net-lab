# clock

`make test` 跑自检。

`sync_add(t_send, t_recv, remote_time)` 摄入一次往返；`sync_estimate(sy, eval_time, &e)`
给出稳健估计。流水线：最小 RTT 过滤（1.5×min + 1 ms，存活的太少则回退到全部低 RTT
样本并置 `rtt_fallback`）→ Theil-Sen 中位数斜率拟合 → 基于 MAD 的离群剔除并重拟合
（残差中位绝对偏差为 0 时只保留相同残差组）。

结果字段：

| 字段 | 含义 |
|---|---|
| `offset` / `slope` | `eval_time` 处的偏移、时钟漂移（秒/秒）；无估计时为 NAN |
| `ci_lo` / `ci_hi` / `ci_half` | 内点残差给出的 95% 置信区间 |
| `asym_bound` | 所用样本最大 RTT / 2，链路非对称带来的物理偏差上界 |
| `err_bound` | `asym_bound + ci_half`，总误差上界 |
| `resid_median` / `resid_p90` / `resid_max` | 内点绝对残差分布 |
| `n_accepted` / `n_selected` / `n_used` / `n_outliers` | 各阶段样本数 |
| `status` / `note` | `EMPTY`、`NEED_MORE`、`OK`；不足时不给单点估计并说明原因 |

摄入会拒绝非有限值、负 RTT、本地发送/接收时间倒退、远端时间倒退，计入 `n_rejected`。

自检用确定性伪随机：抖动路径（误差 < 5 ms、CI 含真值）、100 ppm 漂移、携带低 RTT
的坏时间戳（被 MAD 剔除，且优于朴素均值）、非对称路径（最小 RTT 过滤选中快路径）、
半数脏样本、样本不足、时间戳倒退与窗口上限。

