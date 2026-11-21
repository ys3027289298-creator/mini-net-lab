# snapshot

`make test` 跑自检。

- `src/snapshot.c`：固定步长量化编解码、zigzag + varint tick。
- `src/adaptive.c`：按可见性/距离的自适应量化、误差预算下的比特分配与降级。
  - 步长 `step = base * (1 + d/d_ref) / max(vis, VIS_MIN)`；预算可进一步收紧到 `step <= 2*err_budget`。
  - 位宽按值域/步长上取整，保证量化误差不超预算；超预算先丢速度字段（`RET_DEGRADED`），位置仍超则跳过本 tick（`RET_SKIP`），不压缩位宽硬塞。
  - 步长为零、比特预算为负、可见性/误差预算非法均返回 `RET_BAD`。
  - 自检含固定步长对照，打印带宽下降比例与位置误差（max/mean）。
