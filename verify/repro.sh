#!/bin/sh
# 一键复现弱网一致性核对的全部证据。
# 生成:相位差表、降级开关日志、包体哈希、逐帧动作轨迹(degrade=off),
# 并做确定性校验(连跑两次,证据逐字节一致)与 golden 逐帧一致性校验。
set -e
cd "$(dirname "$0")"
make clean >/dev/null
make >/dev/null
rm -rf out out2
./verify out golden.txt
./verify out2 golden.txt
if diff -r out out2 >/dev/null; then
    echo "deterministic=1"
else
    echo "deterministic=0"
    exit 1
fi
echo "evidence: verify/out/{phase_table.csv,degrade_log.txt,packet_hashes.txt,action_trace_off_tier*.txt}"
