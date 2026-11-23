# mini-net-lab

游戏网络同步基础模块，C 实现，每目录独立构建可自测。

| 目录 | 内容 |
|---|---|
| `snapshot/` | 状态快照量化编解码、变长 tick、v1 兼容的追加式扩展 |
| `clock/` | 时钟偏移估计、最小 RTT 过滤、样本环 |
| `aoi/` | 网格兴趣区域、半径查询、进出差集 |
| `reconcile/` | 预测缓冲、权威校正、回滚计数 |
| `accept/` | 跨版本编码兼容验收：错误码表、兼容矩阵、包体哈希、复现命令（见 `accept/README.md`） |

## 编码兼容约定

快照新增字段后，旧客户端仍要能正确解析位置。四个模块统一采用**不可变 v1
前缀 + 追加式带 tag 扩展**：v1 字节原样保留，新字段只追加在后面；旧客户端
读完 v1 前缀即停止，扩展被忽略。`flags == 0` 时新编码与旧版逐字节相同。

## 测试

```sh
make -C snapshot test && make -C clock test && make -C aoi test && make -C reconcile test
accept/run.sh        # 兼容矩阵 + 逐像素一致 + 包体哈希，产出 accept/evidence/
```
