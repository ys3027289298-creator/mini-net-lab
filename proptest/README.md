# proptest — mini-net-lab 属性测试

对四个模块的真实 C 实现做属性测试：随机生成**延迟、丢包、乱序**的网络场景，
驱动 快照→网络→解码→校正 状态机，每个操作后与独立影子模型比对不变量。
失败自动缩小为最小序列并保存，可一键复现。

## 结构

| 文件 | 作用 |
|---|---|
| `harness.c` | 确定性场景执行器。直接 `#include` 四个模块源码（非副本），逐 op 校验影子模型不变量，违反即打印 `FAIL op=<行> invariant=<名>` 并退出 1 |
| `pbt.py` | 生成器 + ddmin 缩小器 + 复现器 + 人工用例比对 + 变异检查 |
| `manual/*.txt` | 人工构造用例集（四个模块自测的移植 + 对抗用例），头部 `# expect:` 声明预期 |
| `mutants.json` | 8 个人工变异体，用于验证属性测试相对人工用例集零漏报 |
| `failures/*.txt` | 失败的最小序列（含种子、参数、复现命令） |

## 运行

```sh
python3 pbt.py manual                # 人工用例集（含 2 个记录已知缺陷的 expect-fail 用例）
python3 pbt.py run --seed 1 --cases 100   # 随机属性测试，失败缩小并保存到 failures/
python3 pbt.py replay failures/<file>     # 复现最小序列（不变量须一致）
python3 pbt.py mutations             # 变异检查：人工集能杀的，属性测试必须也能杀
make -C proptest test                # manual + run
```

## 模型

- **网络**：每个 `snap` 携带独立随机延迟（`delay_range`）与丢包标志（`loss_prob`）；
  `deliver` 按到达时间冲刷在途包，乱序由延迟方差自然产生。
- **影子模型**：harness 内独立维护 reconcile 环形缓冲/acked/回滚数、aoi 实体表，
  每个 op 后比对模块状态（返回值、acked、count、rollbacks、last_err、查询结果集）。
- **clock 不变量**：合法样本（`t_recv >= t_send`）必须被接受；偏移估计满足
  `|offset - true| <= (dmax - dmin) / 2`（每样本误差 `(d_send - d_recv)/2` 的界）。
- **snapshot 不变量**：`decode(encode(s))` tick 相等、位置误差 `<= 0.5/scale`；
  任意截断前缀不得越界读取。

## 随机种子与参数范围

- 种子：`run --seed S --cases N` 使用种子 `S .. S+N-1`，每种子确定生成一个场景
  （recon/clock/aoi/mixed 按 35/25/20/20 加权）。变异检查固定用种子 1..12。
- 参数范围（`pbt.py` 顶部 `PARAMS`，每次运行总结中打印）：

| 参数 | 范围 |
|---|---|
| 每场景操作数 | 80 |
| 包延迟 | [0, 0.5]s（乱序来源） |
| 丢包率 | 0.25 |
| 位置 x/y | [-50, 50]，量化 scale ∈ {4,16,64,256} |
| 权威误差 | 概率 0.5，幅度 [0.002, 2.0] |
| clock 单向延迟 | [0.01, 0.4]s，真实偏移 [-5, 5] |
| aoi 坐标 | [-100, 100]，半径 15，格子 10，边界命中概率 0.3 |

## 当前发现（基线代码的两个真实缺陷）

1. **clock_accept** — `clock/src/clock.c`：`best_rtt` 只在更小时更新、永不重算。
   最优样本被环形缓冲逐出后，1.5x 过滤可能清空样本集，`sync_add` 对合法样本
   返回 -1，偏移估计冻结。最小序列：`failures/clock_accept-seed9.txt`（17 步）。
   修复方向：逐出时重算 `best_rtt`，或对窗口内最小 RTT 做滑动维护。
2. **aoi_add_capacity** — `aoi/src/aoi.c`：移除的实体不释放槽位，存活数
   小于 `MAX_ENT` 时 `world_add` 仍返回 -1。最小序列：
   `failures/aoi_add_capacity-seed5.txt`（3 步）。修复方向：`world_remove`
   用末位交换压缩数组，或 `world_add` 复用死槽。

两个缺陷均已登记为人工 expect-fail 用例（`manual/clock_eviction.txt`、
`manual/aoi_slot_leak.txt`），修复后将其改为 `expect: pass` 即可作为回归测试。

## 与人工用例集比对（漏报 = 0）

`pbt.py mutations` 对 8 个变异体（reconcile×4、clock×2、snapshot×1、aoi×1）
分别构建变异 harness，比较"人工用例集新杀"与"属性测试新杀"（相对基线的
`(种子, 不变量)` 差集）。当前结果：8/8 双方均杀，**漏报 0**。
