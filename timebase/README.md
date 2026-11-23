# timebase

四个模块共用的单调时间基准。整毫秒 `uint32_t`，只前进，不用系统时钟。

## 接口

- `tb_feed(tb, module, tick)`：模块上报自己观测到的 tick。
  - 低于该模块上次值 → `TB_ERR_BACKWARD`，计入该模块 `rejects`。
  - 低于共享基准但未低于自身 → `TB_OK_LAGGING`，计入 `lags`。
  - 高于基准 → 推进基准。
  - 超过 `TIME_MAX_TICK` → `TB_ERR_RANGE`。
- `tb_read(tb, module, &out)`：从注入源取一个读数再 `tb_feed`。未设置源或
  源为 NULL → `TB_ERR_NULL_SOURCE`，归属读数模块。
- `tb_now(tb, &out)`：读当前共享基准。
- `tb_stat(tb, module)`：该模块的 `accepts / lags / rejects / last_err`。
- `tb_drift_units`、`tb_drift_ppm`：整数漂移计算。

## 验收（`accept.c`）

每个模块跑同一套三阶段：

1. **normal**：正常时长，断言单调、无拒绝，输出漂移数据。
2. **long**：2 小时模拟时长，输出 `src_drift_ms`、`jitter_ms`、`drift_ms`、
   `drift_ppm`。
3. **bad**：空源 / 回退 / 超范围三种非法源，断言各自被拒、计数落在本模块、
   其他模块计数为 0。

另有 **shared** 阶段：四个模块用完全相同的时间源参数（10 ms、100 ppm、
同一 seed）各跑 2 小时，最终 `base` 必须完全一致——这是“统一时间基准”
的跨模块证据，由 `tests/drift-report.sh` 校验。

## 测试不依赖系统时钟

`tests/determinism.sh` 做三件事：

- 静态检查：源码无 `clock_gettime` / `gettimeofday` / `time()` 调用，二进制
  无对应符号导入；
- 复跑：同一输入两次输出逐字节一致；
- 时区：`TZ=Pacific/Kiritimati` 下输出不变。

## 实测漂移（2 小时，源偏 100 ppm）

| 模块 | nominal_ms | actual_ms | src_drift_ms | jitter_ms | drift_ppm | monotone |
|---|---:|---:|---:|---:|---:|---:|
| snapshot | 7200000 | 7425785 | 719 | 225082 | 99 | 1 |
| clock | 7200000 | 7236585 | 719 | 35966 | 99 | 1 |
| aoi | 7200000 | 7272611 | 719 | 71942 | 99 | 1 |
| reconcile | 7200006 | 7309943 | 719 | 109251 | 99 | 1 |

`src_drift_ms=719` 与配置的 100 ppm 一致（7.2e6 ms × 100 ppm ≈ 720 ms）。
各模块 `actual_ms` 的差异来自各自的采样抖动，不是基准差异；统一基准后
四者在 shared 阶段得到同一个 `base=7560712`。
