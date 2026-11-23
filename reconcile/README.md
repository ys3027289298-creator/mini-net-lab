# reconcile

`make test` 跑自检（输出与统一时间基准改造前逐字节一致）。

`make accept` 跑验收：正常 / 2 小时长时间 / 非法时间源。时间来自
`../timebase` 的共享单调基准，不使用系统时钟。
