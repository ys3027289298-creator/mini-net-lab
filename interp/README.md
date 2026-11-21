# interp

远端物体渲染时间轴对齐：按快照到达延迟估计渲染偏置 `off`，
以 `render_time = now - off` 在快照缓冲里插值采样，消除抖动。

- 偏置平滑：slew 限速（`rate` s/s），每帧变化有界
- 误差上界：`interp_error_bound()` 给出采样时间误差上界（跟踪 |目标偏置 - 平滑偏置| 的最大值）
- 偏置突变：到达延迟偏离平滑偏置超过 `jump_thresh` 计数为 jump
- 时间戳回退：小幅回退丢弃为 stale；回退超过 `rebase_gap` 视为服务器时间轴重启，rebase 重建，均不崩溃

`make test` 跑自检。
