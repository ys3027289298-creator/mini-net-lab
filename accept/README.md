# accept — 跨版本编码兼容验收

四个模块（`snapshot` 快照、`clock` 时钟、`aoi` 兴趣区域、`reconcile` 校正）
都新增了字段。历史现象：旧客户端把新字段当成位置的一部分来解析，角色位置乱跳。

本目录证明新编码避免了这件事，并按模块各自验收：

- **旧客户端跑同一段录像，渲出的画面与旧版逐像素一致**（SHA-256 相同）；
- 新编码与旧编码**逐字节兼容**（v1 前缀在 v2 包里原样保留）；
- 每个模块都有**错误码表 / 兼容矩阵 / 包体哈希 / 复现命令**四样证据。

## 一条命令

```sh
accept/run.sh               # 验收；全部通过退出码 0
accept/run.sh --regenerate   # 仅在有意改动线格式时重建录像与 golden
```

首次 clone 后直接跑 `accept/run.sh` 即可（golden 与录像都已提交）。

## 为什么不会再乱跳：v1 前缀不可变

旧版（`0bdd277`）快照线上格式只有三个 varint：

```
tick, zigzag(qx), zigzag(qy)
```

旧客户端读完这三个值就停止。任何新字段只要插在这三个值**之间或之前**，旧
客户端就会把新字段的字节当成 x/y，位置随之乱跳——这正是报告的问题。

新格式把 v1 这三个 varint **原样**作为不可变前缀，新字段一律**追加**在后：

```
v1 前缀 : varint tick, zigzag-varint qx, zigzag-varint qy
扩展    : { varint tag, varint len, payload }  ... 可重复
```

- `flags == 0`（常见情况）不写扩展，整包与旧版**逐字节相同**；
- 旧客户端读到的仍是同样的 tick/x/y，扩展字节被它忽略；
- 未知 tag 按长度跳过并计数，未来再加字段也不会破坏本版解码器。

四个模块的统一约束：**只允许在 v1 体之后追加带 tag 的记录，永不插入 v1
体内部**。每个模块的 v1 体在各自头文件里写死了布局。

| 模块 | v1 体（不可变前缀） | v2 追加 |
|---|---|---|
| `snapshot` | tick, qx, qy | `flags`（tag 1）、`version`（tag 0） |
| `clock` | n、n×{rtt,offset,at}、best_rtt、offset | `filtered`（样本数） |
| `aoi` | n、n×{id,x,y,alive}、cell、radius | `rev`（世界版本号） |
| `reconcile` | n、n×{tick,x}、acked、rollbacks、last_err | `ack_hits` |

## 四样必备证据（缺一即 `ACCEPTANCE=FAIL`）

| # | 证据 | 文件 | 产生方式 |
|---|---|---|---|
| 1 | 错误码表 | `evidence/error_codes.md` | 从各头文件的 `*_ERR_LIST(X)` X-macro 展开；enum、`*_err_name()` 与本表同一份列表，运行时断言名字一致 |
| 2 | 兼容矩阵 | `evidence/compat_matrix.csv` | `accept.c` 实测每个 生产者×消费者 组合；行数不足或任一行 FAIL 即失败 |
| 3 | 包体哈希 | `evidence/body_hashes.csv` + `golden/bodies.sha256` | 每模块 v1 参考体 / 当前 v1 前缀 / 当前 v2 全包 的 SHA-256，逐条对 golden |
| 4 | 复现命令 | `evidence/repro.log` | `run.sh` 生成，逐模块 `make test`、驱动、两个客户端、golden 校验 |

另有两份：

- `evidence/pixel_identity.csv`：录像重放的帧缓冲 SHA-256 与逐字节差异数；
- `evidence/two_client.log`：**两个独立二进制**（旧客户端只链旧编解码，
  新客户端链当前模块）读同一录像的重放结果。

## 兼容矩阵覆盖的每个格子

`run.sh` 要求矩阵**恰好** 23 行且全部 PASS（少一行即失败）：

- 每模块 3 格：
  - `v1 → 当前`：旧数据被新代码读，逐字段一致；
  - `v1 → pinned_v1`：当前编码器对无新字段的状态，产出与旧版**逐字节相同**；
  - `v2 → pinned_v1`：旧客户端读带新字段的包，前缀值不变、尾部被忽略；
- clock / aoi / reconcile 各 1 格 `v1(无新字段) → 当前`：纯 v1 体解码后，
  v2 专属字段必须保持零值（不被凭空造出）；
- `snapshot` 错误路径 2 格：扩展被截断 / 扩展版本越界时，**前缀仍然可用**，
  位置不回退、不乱跳；
- 两个客户端重放 3 格；
- 旧版参照 1 格：同一会话的纯 v1 录像 vs 新录像，由**旧客户端**分别重放，
  帧缓冲必须一致；
- 负向对照 2 格：把新字段插到 x/y **之前**（即当初的错误做法），断言画面
  **必然不同**——证明像素比较不是恒真。

## 旧客户端为什么会渲出相同画面

`accept/legacy/`：

- `snapshot_v1.c` 是 `0bdd277:snapshot/src/snapshot.c` 的**逐字节拷贝**（run.sh
  从 git 重新导出并 `cmp` 校验）；
- `legacy_v1.c/h` 给出 clock/aoi/reconcile 的 v1 状态体参考编解码（基线没有
  线格式，这里就是它们的 v1 契约）。

`accept/tools/client_old.c` 只链接旧编解码，`client_new.c` 只链接当前模块；
两者读同一 `recordings/replay.bin`（内含纯 v1 包与带扩展的 v2 包），用**同一
个整数渲染器**（`render.c`）把位置画进固定帧缓冲，比较 SHA-256。

## 判别力（负向自测，已实测）

- 改 `golden/bodies.sha256` 一个字节 → `HASH DIFF` → FAIL；
- 让旧客户端某帧偏 1 像素 → `framebuffer bytes: DIFFER` → FAIL；
- 把新字段插到 x/y 之前（复现原始 bug）→ `v2 → pinned_v1` FAIL，且重放差异
  出现在帧缓冲上 → FAIL。

## 目录

```
accept/
  run.sh                 端到端编排 + 四样证据硬门禁
  accept.c               兼容矩阵 / 错误码表 / 包体哈希 / 像素身份
  render.c/h             确定性整数帧缓冲 + 自带 SHA-256
  recording.c/h          录像文件读写（两个客户端共用）
  legacy/
    snapshot_v1.c        0bdd277 快照编解码原样拷贝（旧客户端口径）
    legacy_v1.c/h        四模块 v1 状态体参考编解码
  tools/
    client_old.c         旧客户端：只链旧编解码，渲出帧缓冲哈希
    client_new.c         新客户端：链当前模块，渲出帧缓冲哈希
    make_recording.c     生成/复现录像
  recordings/            提交的录像（replay.bin、replay_naive.bin、SHA256SUMS）
  golden/bodies.sha256    提交的包体哈希
  evidence/              每次运行产出（构建产物，不入库）
  build/                 临时构建目录（不入库）
```
