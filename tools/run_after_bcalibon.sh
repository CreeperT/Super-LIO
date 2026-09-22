#!/usr/bin/env bash
set -u
cd /home/creepert/Super-LIO
while pgrep -f "[r]un_gated.sh b 2 bcalibon" >/dev/null 2>&1; do sleep 30; done
sleep 15
python3 tools/analyze_dt_attribution.py > exp_repeat/dt_attribution.txt 2>&1
echo "=== 分析完成 $(date +%H:%M:%S) ===" >> exp_repeat/dt_attribution.txt
