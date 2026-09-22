#!/usr/bin/env bash
# 串行接续：等 bpitch5t 队列让出 2-9 核后再跑，避免两队列争核导致双双判无效。
# 用途：δt 归属验证——无注入 + calib ON（自然失配档），与 bpitch5t（+5° 注入）对照，
#       分离 δt2 中"时基残差"与"旋转误差泄漏"两种成分；同时把表 VIII 时偏行的 n 提到 3。
set -u
cd /home/creepert/Super-LIO
LOG=exp_repeat/bcalibon_queue.log
# [r] 技巧：避免 pgrep 匹配到本脚本自身的命令行
while pgrep -f "[r]un_gated.sh b 3 bpitch5t" >/dev/null 2>&1; do sleep 30; done
echo "=== bpitch5t 已结束，开始无注入 calib-ON 对照 $(date +%H:%M:%S) ===" >> "$LOG"
bash tools/run_gated.sh b 2 bcalibon reloc on none 30 4 1.0 >> "$LOG" 2>&1
echo "=== 全部结束 $(date +%H:%M:%S) ===" >> "$LOG"
