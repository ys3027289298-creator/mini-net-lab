# snapshot

版本化状态快照编解码：varint 版本头 + 按版本 schema 逐字段解析。

- 版本缺失或超出 `[SNAP_VERSION_MIN, SNAP_VERSION_MAX]`：直接拒绝，不猜版本
- 字段在 varint 中间被截断：报错并指出字段名与版本号
- 尾部字段在干净边界缺失：取 schema 默认值并计数
- 超出 schema 的尾部未知字段：跳过并计数字段数与字节数

`make test` 跑自检。
