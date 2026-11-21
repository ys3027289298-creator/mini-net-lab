# fielddep

字段依赖图：标记可推导字段，接收端按依赖序重算，替代重复传输。

- `fd_add_base` / `fd_add_derived` 注册字段，派生字段带规则、依赖列表和误差上界
- `fd_build` 拓扑排序给出重算顺序；依赖成环返回 -1 并在 `cycle[]` 列出环上字段，依赖缺失在 `missing[]` 列出
- `fd_derive_all` 按序重算；`fd_check` 逐字段对比全量传输，给出推导误差
- `fd_full_bytes` / `fd_wire_bytes` 给出带宽下降

`make test` 跑自检。
