# mini-net-lab

游戏网络同步基础模块，C 实现，每目录独立构建可自测。

| 目录 | 内容 |
|---|---|
| `snapshot/` | 状态快照量化编解码、变长 tick |
| `clock/` | 时钟偏移估计、最小 RTT 过滤、样本环 |
| `aoi/` | 网格兴趣区域、半径查询、进出差集 |
| `reconcile/` | 预测缓冲、权威校正、回滚计数 |
| `common/` | 统一帧计时库 `ftrace`（单时钟源、重叠/对账/上界，编译开关 `FT_ENABLE`） |
| `tools/` | `collect_evidence.sh` 一键收集分项耗时表、重叠区间日志、总和对账与复现命令 |

## 构建

```bash
make -C snapshot  test          # 默认：计时关闭，输出与原始版本一致
make -C snapshot  test-timing   # -DFT_ENABLE=1：统一口径，输出计时报表与自检验证
tools/collect_evidence.sh       # 全量验收，证据写入 artifacts/timing/report.md
```

