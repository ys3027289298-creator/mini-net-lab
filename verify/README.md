# verify

弱网一致性核对台架:权威端角色跑固定动作脚本,客户端经有损链路跟踪远端角色,
按丢包率分档(0/5/10/20/30%)测量**动作相位差**(本地远端角色相位 vs 权威相位,单位 tick)。

降级策略:客户端滑窗(16 个发送机会)估丢包,>15% 时冻结预测(保持上一姿态)且服务端
降为 1/4 发包率,<5% 时恢复(滞回)。`make test` 跑自检并生成证据。

## 四样独立证据(均在 `out/`)

| 文件 | 内容 |
|---|---|
| `phase_table.csv` | 相位差表:丢包档 × 降级 off/on × 降级前(pre)/后(post),含均值/最大相位差、回滚数、发包数 |
| `degrade_log.txt` | 降级开关日志:每次切入/切出降级的 tick 与滑窗丢包率 |
| `packet_hashes.txt` | 每个发包包体的 fnv1a-64 哈希(tier/mode/seq/tick/len) |
| `../repro.sh` | 直接可跑的复现脚本:重新构建、连跑两次并 diff 全部证据 |

另生成 `action_trace_off_tier*.txt`:degrade=off 的逐帧动作轨迹。

## 逐帧一致性

- `golden.txt` 记录 degrade=off 各档轨迹哈希;`make test` 校验 `golden_match=1`,
  即关闭降级时动作与当前行为逐帧一致。
- `frame_identical_tier0=1`:0% 丢包下降级机制不触发,on/off 轨迹哈希相同。
- `deterministic=1`:同种子重放,轨迹与包流哈希不变。
