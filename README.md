# mini-net-lab

游戏网络同步基础模块，C 实现，每目录独立构建可自测。
每个模块把失败收敛为命名错误码并提供确定性故障注入，`accept/` 做失败隔离验收。

| 目录 | 内容 |
|---|---|
| `snapshot/` | 状态快照量化编解码、变长 tick |
| `clock/` | 时钟偏移估计、最小 RTT 过滤、样本环 |
| `aoi/` | 网格兴趣区域、半径查询、进出差集 |
| `reconcile/` | 预测缓冲、权威校正、回滚计数 |
| `accept/` | 失败隔离验收：错误码表、注入日志、包体哈希、复现命令（见 `accept/README.md`） |

## 测试

```sh
make -C snapshot test && make -C clock test && make -C aoi test && make -C reconcile test
accept/run.sh        # 端到端失败隔离验收，产出 accept/evidence/
```
