# replay

输入回放 + 裁剪，两条路径：

| 路径 | 语义 |
|---|---|
| `replay_naive` | 现状：按远端 tick 裁剪，在本地**接收时刻**应用，位置直接吸附/保持，允许突跳，不接触任何对齐代码 |
| `replay_aligned` | 默认：时钟偏移对齐（复用 `clock/` 的 min-RTT 估计器）→ 裁剪点映射到本地时间线 → 跨越裁剪点的包按轨迹插值切开 → 速度上限夹逼追赶，轨迹不许跳 |

`make test` 跑 18 项自检（含关闭对齐的逐字节稳定性、FNV-1a 包体校验、
30 ms 抖动重对齐、末尾收敛）。`make csv` 输出两条路径的逐帧 CSV。

材料与复现步骤见 `materials/repro.md`，三份数据材料由
`materials/gen.py` 单一来源生成，`materials/verify.py` 校验它们与
`src/replay.c` 内嵌夹具一致（`make verify`）。
