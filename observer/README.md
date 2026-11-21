# observer

多观察者一致性：每个观察者按 AOI 半径算可见集并维护订阅副本，交叉核对同一实体的
状态，与全量广播基线比对，掉线时清理订阅。

- `net_visible` / `net_sync`：权威可见集查询，按进出差集协调订阅
- `net_cross_check`：两观察者共同可见实体逐侧对权威状态核对，差异定位到
  `{observer, entity, kind}`，`kind` 为 `DIFF_STATE` / `DIFF_MISSING` / `DIFF_EXTRA`
- `net_full_broadcast` / `net_compare_broadcast`：全量广播参考视角与基线比对
- `net_disconnect`：掉线释放订阅与槽位，同 id 重连不积累

`make test` 跑自检。
