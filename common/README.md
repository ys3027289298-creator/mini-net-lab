# common — 统一帧计时库 ftrace

四个模块共用的帧计时口径，头文件 `ftrace.h`（header-only）。

## 编译开关

- 默认构建（各模块 `make test`）：`FT_ENABLE=0`，所有插桩 API 是 `static inline` no-op，
  调用点不生成任何计时代码（可对二进制 `nm` 验证无 `ft_*`/`clock_gettime` 符号），
  模块自检输出与引入前逐字节一致。
- 计时构建（各模块 `make timing` / `make test-timing`）：`-DFT_ENABLE=1`，
  使用 `CLOCK_MONOTONIC` 单一时钟源。

## 口径

- span 为半开区间 `[start, end)`，按 start 时刻归属帧；帧由 `ft_frame_begin/end` 界定。
- 每帧：`span_sum = Σ(end-start)`；`union` 为区间并集长度；
  `overlap = span_sum - union`（重叠按多重度计费）；`gap = frame_total - union`。
- 恒等式：`span_sum - frame_total == overlap - gap`。
- 上界（逐帧计算再求和）：`|diff| <= overlap + gap + eps + resolution`，
  `eps` 为浮点舍入预算，`resolution` 为时钟分辨率（注入虚拟时钟时为 0）。
- `ft_verify()` 校验全部恒等式与上界；`ft_print_report()` 输出分项表、重叠日志、对账块。

## 自测

```bash
make -C common clean test
```

- `ftrace_test.c`：注入虚拟时钟的精确场景（兄弟重叠、嵌套、不相交），期望值精确到 0，
  另含篡改报表的负向验收和槽位耗尽边界。
- `ftrace_noop_test.c`：不带 `-DFT_ENABLE` 编译，验证 no-op 接口干净可用。
