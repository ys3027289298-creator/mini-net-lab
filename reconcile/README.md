# reconcile

`make test` 跑自检。

客户端每 tick 消费一条输入并预测一个状态，输入与预测保存在重放窗口里。
收到未压缩的权威状态（tick A）时：

1. A 已跌出重放窗口 → 返回 `RECON_WINDOW`，计入 `window_short`，不丢帧、不改动本地状态；
2. 与本地预测比对，回滚到 A 的权威状态，重放 A+1..当前 tick 的输入；
3. 校正量大于 `snap_threshold` → 立即吸附（`RECON_SNAP`）；
   否则保持当前可见位置，用渲染偏移在 `smooth_ticks` 内插值抹平（`RECON_SMOOTH`）。

自检用全速率、未压缩的权威序列做对照，输出位置误差上界与回滚次数：

- `max_pos_err` / `position_error_bound`：渲染位置相对权威序列的最大误差；
- `replays` / `rollbacks` / `snaps` / `smooths` / `matches`：重放与校正分项计数；
- `window_short` / `stale` / `future`：窗口不足、重复、超前帧的计数；
- `max_smooth_jump` / `max_snap_jump`：校正瞬间的可见位移（平滑应约为 0）。

每个权威帧只被归类一次：`auth_seen == matches + rollbacks + window_short + stale + future`。
