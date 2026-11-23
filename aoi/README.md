# aoi

`make test` 跑自检（成功路径输出与 `tests/golden/aoi.out` 逐字节一致）。

失败统一经 `mnl/mnl.h` 的 sink 上报，批量入口为 `mnl_aoi_run`：
每条 entry 独立验收，坏输入只记录 `(module, field, reason, entry_index)`
并隔离在该条目内，不影响其余条目；载荷按指针透传，runner 内零复制。
