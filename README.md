# mini-net-lab

游戏网络同步基础模块，C 实现，每目录独立构建可自测。

| 目录 | 内容 |
|---|---|
| `snapshot/` | 状态快照量化编解码、变长 tick |
| `clock/` | 时钟偏移估计、最小 RTT 过滤、样本环 |
| `aoi/` | 网格兴趣区域、半径查询、进出差集 |
| `reconcile/` | 预测缓冲、权威校正、回滚计数 |
| `mnl/` | 四模块共用的统一失败上报（sink）与批量 runner 契约 |
| `tests/` | 失败矩阵、隔离/定位测试、零拷贝证据、golden 基线 |

## 统一失败上报与隔离

四个模块原来各自返回 `-1`，调用方无法区分错误来源。现在所有校验点都
经过同一个 sink（`mnl/mnl.h`）：

- 每条失败记录是四元组 `(module, field, reason, entry_index)`，跨模块失败
  可直接定位到模块与具体字段（`mnl_format` 输出
  `clock/t_recv/range@entry1` 这样的定位串）。
- 批量入口 `mnl_<module>_run(state, sink, items, n, results)` 对每条 entry
  独立验收：坏输入只产生一条记录并标记对应 result，不修改模块状态，不
  中断后续 entry。
- `reason` 区分 `null / range / nonfinite / duplicate / not_found / full /
  parse / bad_op`；个别沿用原语义的软失败（reconcile 的 ack 未命中返回 0）
  也会留记录，可在 `result.err` 与 sink 中同时查到。
- 模块状态由调用方拥有（`mnl_state_t`，编译期 `_Static_assert` 保证容量），
  库内不做任何动态分配。

## 零拷贝契约

runner 只按指针透传调用方的载荷结构（state/sample/point/idset），不复制
载荷；唯一允许的写操作是结果元数据与调用方提供的输出缓冲。证据有三层，
`tests/check.sh` 全部自动执行：

1. **静态扫描**：词法确认所有 `mnl_*_run` 函数体内不存在
   `memcpy/memmove/mempcpy`（clock 环淘汰的 memmove 位于原语 `sync_add`，
   操作的是自身 Sample 数组而非载荷）。
2. **LD_PRELOAD 拦截**（`mnl_zc_shim.so`）：注册全部载荷地址区间，运行
   真实批量时任何触及这些区间的复制例程立即 `abort`。含 glibc 版本化符号
   （`memcpy@GLIBC_2.14` 等）的别名处理。
3. **负向对照**（`mnl_zc_negative`）：故意复制一次注册载荷，验证拦截器
   确实以 SIGABRT 终止；拦截器若失效，验收失败。

## 验收

```
make            # 顶层：四个模块自检 + 统一测试套件
./tests/check.sh
```

`check.sh` 依次验证：全量构建、四个模块自检输出与 `tests/golden/` 基线
**逐字节一致**（成功路径行为零变化）、静态零拷贝扫描、77 项运行期检查、
拦截器正向与负向证据。
