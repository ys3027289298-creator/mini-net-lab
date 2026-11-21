# coalesce

AOI 进出事件的帧内合并与批投递。`make test` 跑自检。

- 按目标合并：同一目标的多次 enter/leave 在帧窗口内折叠，互抵的事件直接消去，净效果与逐条投递一致。
- 帧窗口批处理：`coal_event` 累积，`coal_flush` 关窗输出 delta 并清空 pending；权威 presence 集合跨窗保留。
- 超容量降级：待发 delta 条数超过 `capacity` 时，本窗改发全量快照，客户端以快照替换本地集合。
- 容量为零（或越界）：`coal_init` 返回 -1。
- 丢事件计数：事件超出内部 slot 表时计入 `dropped`，并强制本窗降级快照，状态仍与逐条投递一致。
- 压缩比例：每窗 `reduction = 1 - 合并后条数 / 原始条数`，另由 `coal_reduction` 给累计值。
