# aoi

`make test` 跑自检。

网格兴趣区域 + 空间哈希：

- 实体按所在网格单元 `(cx, cy)` 哈希到桶，链式冲突；
- 查询只枚举半径覆盖的单元所对应的桶，同一个桶（多个单元命中同一桶时）只遍历一次；
- 结果与全场遍历 `world_query_brute` 逐项一致（测试里对多组坐标/半径交叉比对）；
- 桶数按负载因子 `live / cap` 扩容（8 → 2048，2 的幂），扩容重建后查询结果不变；
- `max_load <= 0` 时 `world_init` 返回 -1，之后 `world_add` 也拒绝。

桶函数 `bucket = (cell * 2654435761u) & (cap - 1)`：cap 为 2 的幂且整除
`GRID*GRID`，奇乘子对 2^k 取模是双射，故每个桶分到的不同的网格单元数恰好是
`ceil(GRID*GRID / cap)`。

于是链长有可验证的上界：

```
max_chain <= min(live, ceil(GRID*GRID / cap) * max_cell_occ)
```

其中 `max_cell_occ` 是单个网格单元里存活实体的最大数。`world_stats()` 输出
每桶链长直方图、冲突桶数、实测最坏链长以及这个理论上界，测试断言实测值不超过
上界；在高负载因子下（桶少于单元数）上界与实测都收敛到 `max_cell_occ`。
