# 复现步骤（material 4）

输入回放与裁剪的时序不一致导致远端位置突跳。本目录用一个确定性夹具
（20 Hz tick、远端时钟领先本地 +2.0 s、8 个各 5 tick 的包，其中 4 号包
接收方向多 30 ms 抖动）复现该问题，并验证两条回放路径。

## 夹具与三份数据材料

| 文件 | 角色 | 时钟域 |
|---|---|---|
| `traj_samples.csv`  | 材料 1：轨迹采样，每 4 tick 一个点 | 远端时钟 `remote_t` |
| `timing_records.csv`| 材料 2：逐包发送/接收时刻与远端事件时刻 | 本地时钟 + 远端时钟 |
| `packet_summary.csv`| 材料 3：包体摘要（tick 区间、量化端点、长度、FNV-1a 校验） | 按 seq 与材料 2 对齐 |
| `gen.py` | 以上三份 CSV 的唯一事实源 | — |

裁剪点固定为 **tick 12**：2 号包（tick 5..9）完全在裁剪点之前，
3 号包（tick 10..14）跨越裁剪点。

重新生成材料（隔离模式，不在数据目录里跑解释器）：

```sh
cd /tmp && python3 -I <repo>/replay/materials/gen.py
```

## 1. 复现突跳（关闭对齐 = 现状）

```sh
make -C replay test          # 自检，legacy_jumps=1 即复现突跳
./replay/replay csv | awk '/^# naive/{p=1} /^# aligned/{p=0} p'
```

旧路径按本地**接收时刻**应用裁剪后幸存的包。第一段输出中：

```
98.6850,2.250000,...        # 裁剪前最后位置（2 号包端点，保持）
98.7350,3.500000,...        # 3 号包到达：单帧 x 跳变 1.25（25 个量化单位）
```

4 号包因为 30 ms 接收抖动，落点也被整体推后（自检 `jitter_realigned`
记录旧路径落在帧 6）。

## 2. 时序对齐 + 禁止突跳（默认路径）

同一输入走 `replay_aligned`，自检覆盖全部契约：

- `sync_offset=1`：最小 RTT 过滤的偏移估计精确还原 +2.0 s，带 30 ms
  抖动的样本被 1.5× min-RTT 门限排除（与 `clock/` 同一估计器）；
- `cut_on_grid=1` / `cut_entry_x=1`：裁剪点映射到本地时间线 98.6000，
  跨越裁剪点的 3 号包在该点用轨迹分段线性插值**切开**，入口位置
  x=3.000000（tick 12 的真实位置），不是 2 号包的 2.25；
- `aligned_no_jump=1`：每帧位移被速度上限
  `1.05 × 轨迹最大采样速度 × tick`（再加一个量化单位容差）夹住，
  CSV 中相邻帧位移连续，无任何单帧突跳；
- `jitter_realigned=1`：4 号包按其远端事件时刻落在帧 7，抖动不再
  移动落点；
- `converges=1`：末尾保持帧让速度受限的追赶精确收敛到最后一个目标；
- `trim_respected=1`：被裁掉的 1/2 号包永远不会成为裁剪后的目标；
- `crc_tamper_detected=1` / `bad_pkt_counted=1`：篡改包体摘要后
  FNV-1a 校验失败，坏包被跳过且回放仍然连续。

## 3. 关闭对齐后结果与现在一致

对齐开关就是路径选择，不存在共享状态：

```c
replay_naive(pkts, recv, n, cut, ...)    // 关：旧语义，不接触 Sync/Traj
replay_aligned(pkts, n, cut, ...)        // 开：对齐 + 速度夹逼
```

自检 `off_path_stable=1` 对同一份输入连跑两次旧路径并逐字节 `memcmp`
帧缓冲，结果完全相同；`make test` 中旧路径的所有数值断言（27 帧、
保持起点 2.25、突跳 >1.0）独立于对齐代码。

## 4. 材料与代码一致性

```sh
cd /tmp && python3 -I <repo>/replay/materials/verify.py
```

校验脚本重算三份 CSV 的 FNV-1a 与量化值，并与 `src/replay.c` 内嵌夹具
的常量逐一比对，任何漂移都会非零退出。
