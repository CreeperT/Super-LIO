#!/usr/bin/env bash
# 全量 1× 复跑队列（按用户决定：B 段按 1× 重收；统一使用正确平移的纯旋转注入）。
# 参数顺序：<seg> <n> <label> [mode] [calib] [inj] [pitch] [max_try] [rate]
# 门控：收敛 / 无初始化期退化抑制 / 覆盖率≥95% / 时长≥95%；早期失配轮次自动提前中止。
set +u
cd /home/creepert/Super-LIO || exit 1
R=1.0

run() { local n=$1 lab=$2; shift 2
  echo "############ $(date +%H:%M:%S) $lab (n=$n, $*) ############"
  bash tools/run_gated.sh "$@" || echo "!!! $lab 未达目标，继续下一项"; }

# —— 优先级 1：B 段剂量系列（论文第四节的核心结论）——
run 5 b30 b 5 b30 reloc off none 30 20 $R    # 已有 1× 有效 2 次（1.151/1.153）
run 3 b31 b 3 b31 reloc off none 31 12 $R
run 3 b33 b 3 b33 reloc off none 33 12 $R

# —— 优先级 2：注入组（沿用 b2.yaml 原错误外参预设：旋转 + 平移，与 A 段既有注入同口径）——
run 3 bpitch5 b 3 bpitch5 reloc on pitch5 30 10 $R
run 3 byaw5   b 3 byaw5   reloc on yaw5   30 10 $R
run 3 apitch5 a 3 apitch5 reloc on pitch5 30 10 $R
run 3 ayaw5   a 3 ayaw5   reloc on yaw5   30 10 $R

# —— 优先级 3：SLAM 基线（米级误差，门控阈值 6 m，Umeyama 对齐）——
run 3 bslam b 3 bslam slam off none 30 10 $R
run 3 aslam a 3 aslam slam off none 30 10 $R

echo "############ 1× 队列结束 $(date +%H:%M:%S) ############"
