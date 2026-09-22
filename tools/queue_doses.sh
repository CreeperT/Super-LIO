#!/usr/bin/env bash
# 排队等待当前链路结束后，续跑 B 段剂量系列（干净口径，各 3 次有效）。
# 之所以单独成文件：本脚本自身的命令行不含 "run_gated.sh"，pgrep 才不会匹配到自己。
set +u
cd /home/creepert/Super-LIO || exit 1

while pgrep -f "[r]un_gated.sh" >/dev/null; do sleep 60; done
echo "=== 前序链路结束，开始剂量系列 $(date +%H:%M:%S)"

bash tools/run_gated.sh b 31 3 b31
bash tools/run_gated.sh b 33 3 b33

echo "=== 剂量系列完成 $(date +%H:%M:%S)"
