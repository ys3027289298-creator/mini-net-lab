# accept — 失败隔离验收

四个模块（`snapshot` 快照、`clock` 时钟、`aoi` 兴趣区域、`reconcile` 校正）
出错时不再让客户端崩溃：每个模块把失败收敛为**命名错误码**，并提供
**确定性故障注入点**。本目录对这一契约做端到端验收。

## 一条命令

```sh
accept/run.sh            # 验收，全部通过时退出码 0
accept/run.sh --regenerate   # 仅在成功路径线格式有意变更时重建 golden
```

## 四样必备证据（缺一即未完成）

`run.sh` 末尾有硬门禁，四样证据任一缺失或为空即 `ACCEPTANCE=FAIL`：

| # | 证据 | 文件 | 由谁产生 |
|---|---|---|---|
| 1 | 错误码表 | `evidence/error_codes.md` | `accept.c` 从各模块头文件枚举经 `*_err_name()` 运行时解析，枚举是唯一事实源 |
| 2 | 失败注入日志 | `evidence/injection.log` | `accept.c`，含每轮注入的期望/观测码与 4×4 隔离矩阵 |
| 3 | 包体哈希 | `evidence/body_hashes.txt` + `golden/*.body` | `run.sh` 对 git-HEAD 原始源码与重构源码各自捕获包体并 sha256 比对 |
| 4 | 复现命令 | `evidence/repro.log` | `run.sh` 生成，逐模块 `make test` 与整套 `accept/run.sh` |

另有两类佐证（不计入“四样”，但同样在日志里）：

- `evidence/no_copy.log`：**动态**无二次载荷拷贝证据——调用方 buffer 地址
  操作前后一致；
- `evidence/no_copy_audit.log`：**静态**审计（`audit_no_copy.sh`）——无
  malloc/calloc/strdup 等，所有 memcpy/memmove 走带注释白名单。

## “成功路径字节不变”如何保证

1. `run.sh` 用 `git show HEAD:<m>/src/<m>.c` 导出**重构前**原始源码到隔离目录；
2. 双模式捕获程序 `tools/capture_*.c` 对原始源码（`-DML_PRISTINE`，文本包含
   其 static 实现）和重构对象（公开 `*_` API）构造**同一个规范化包体**；
3. 两侧 sha256 逐模块比对，必须完全一致；golden 字节随 `golden/` 提交，普通
   验收还会再跟提交的 golden 做 `sha256sum -c` 与 `cmp`。

## “四次注入只影响对应条目”如何测量

每轮注入使用**独立全新状态**，并做两种测量：

- **对角线（自身条目）**：武装故障前对目标对象做整对象内存快照，故障返回后
  逐字节比对。故障钩子位于任何写操作之前，因此前后必须字节一致
  （`state_unchanged=1`）；
- **非对角线（其它条目）**：在独立新状态上重算其余三模块的成功包体，必须仍与
  基线哈希一致。

4×4 矩阵全部为 1 才算隔离成立。

## 判别力（负向自测）

本验收不是恒真：

- 改动 `golden/clock.body` 一个字节 → `BODY DIFF` → `ACCEPTANCE=FAIL`；
- 让时钟的注入故障额外写 `sy->n = 999` → 对角线 `state_unchanged=0`、
  `isolated=0` → `ACCEPTANCE=FAIL`。

## 目录

```
accept/
  minilab.h            规范化、定长、无填充的包体序列化（两端共用）
  accept.c             验收驱动：错误表 / 注入矩阵 / 动态无拷贝
  audit_no_copy.sh     静态无二次拷贝审计
  run.sh               端到端编排与证据完整性硬门禁
  tools/capture_*.c    双模式包体捕获（pristine 文本包含 / current 链接对象）
  golden/              提交的原始源码 golden 包体与 sha256
  evidence/            每次运行产出的证据（构建产物，不入库）
  build/               临时构建目录（不入库）
```
