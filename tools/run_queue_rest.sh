#!/usr/bin/env bash
# 补齐队列：b30/b31/b33 已完成，剩余三组因传参错位与就绪标志不匹配未跑成。
# 参数顺序（务必对齐）：<seg> <n> <label> [mode] [calib] [inj] [pitch] [max_try] [rate]
set +u
cd /home/creepert/Super-LIO || exit 1
RATE=0.7

run() { local n=$1 lab=$2; shift 2
  echo "############ $(date +%H:%M:%S) $lab (n=$n, $*) ############"
  bash tools/run_gated.sh "$@" || echo "!!! $lab 未达目标，继续下一项"; }

run 2 bpitch5 b 2 bpitch5 reloc on  pitch5 30 8 $RATE
run 2 byaw5   b 2 byaw5   reloc on  yaw5   30 8 $RATE
run 2 bslam   b 2 bslam   slam  off none   30 8 $RATE

echo "############ 补齐队列结束 $(date +%H:%M:%S) ############"
