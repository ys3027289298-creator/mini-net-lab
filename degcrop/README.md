# degcrop

远端动画降级与 AOI 裁剪联动模块，`make test` 跑自检并打印复现日志。

## 问题

远端实体被 AOI 裁剪出视野后，动画从高精度逐 tick 流（fine）降级为稀疏粗量化
包 + 速度外推流（degraded）。不联动时，AOI 边界上的每次参考点变更都直接套用到
渲染位置：

- 出界瞬间还没有 coarse 包，降级参考是陈旧值（初始为 0），渲染点硬 snap；
- 周期性 coarse 包重同步、重入 AOI 的校正同理，都会产生位置瞬移。

## 联动方案（linked=1，默认）

- 边界切换为降级时，用最后渲染位置在**接收端本地播种**降级参考（不产生网络包），
  外推从最后可见位置起步，不留陈旧窗口。
- 任何参考点变更（模式切换或 coarse 重同步）都启动追赶混合：每 tick 渲染点向
  目标参考点最多移动 `DC_MAX_STEP`（0.25 世界单位），到达前轨迹步进被硬上界约束，
  因此任何速度变化或反向都不可能出现跳变。
- 混合纯接收端完成，线上字节数与关闭联动时**完全一致**（自检 `packet_parity_*`
  断言两种模式包数、字节数相等）。
- 跳变判定：渲染位置单 tick 步进超过 `DC_JUMP_EPS`（2.0，高于实体最大物理步速
  1.5）记一次 jump——正常变速不是瞬移，硬不连续才是。

## 关闭联动（linked=0）

`dc_init(&w, 0)` 回到现状：AOI 边界硬切模式、coarse 重同步硬 snap，降级参考
保持陈旧。自检 `unlinked_reproduces` 断言该路径至少出现 2 次 jump（演示运行中
实测 4 次，最大单 tick 步进 86）。

## 场景

折线匀速运动，AOI 成员关系按时间线确定性给出，三次边界切换精确落在
tick 25（出）、50（入）、75（再出），另有一个 AOI 抖动（flap）子场景验证
混合中途反复进出仍无跳变。

## 复现方式

```sh
cd degcrop
make test
```

输出包含：

- `[switch]`：每次模式切换的 tick、实体、`fine->degraded`/反向方向、linked 开关。
- `[traj]`：每 5 tick 及边界 tick 的轨迹采样（AOI 状态、模式、混合标记、
  渲染位置、真实位置）。
- `packet summary`：fine/coarse 包数、总字节、平均包体；两种 linked 取值逐字节
  相同。
- 自检项：`linked_no_jump`、`linked_step_bounded`、`unlinked_reproduces`、
  `unlinked_worse`、`packet_parity_pkts/bytes`、`linked_flap_safe`、
  `unlinked_flap_teleports`。
