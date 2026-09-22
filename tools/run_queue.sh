#!/usr/bin/env bash
# 干净口径复跑队列（按重要性排序，任一步失败不阻断后续）。
# 全部经过 tools/gate_run.py 门控：收敛 / 无初始化期退化抑制 / 覆盖率≥95% / 时长≥95%。
#
# 回放速率取 0.7：实测处理速率在 15.5–29 Hz 之间波动，低于约 18 Hz 时节点丢失地图匹配
# （knn_valid→0）并发散——那批失败与数据无关，是机器负载所致。降速后节点所需吞吐降至
# 名义的 70%，消息时间戳不变，故轨迹时间轴与 1× 一致。已验证：降速前两轮 B 段 1×
# 有效运行给出 1.153 / 1.151，与降速后的结果可直接对比。
set +u
cd /home/creepert/Super-LIO || exit 1

RATE=0.7

run() {   # $1=n_valid $2=label $3.. = 其余参数（末位为回放速率）
  local n=$1 lab=$2; shift 2
  echo "############ $(date +%H:%M:%S) $lab (n=$n, $*) ############"
  bash tools/run_gated.sh "$@" || echo "!!! $lab 未达目标，继续下一项"
}

# A 段关标定已在 1× 下完成 5 次有效，不再复跑

# 1) B 段关标定自然失配（已有 2 次有效：1.153 / 1.151）
run 5 b30   b 5 b30   reloc off none 30 20 $RATE
# 2) B 段受控剂量（纯俯仰失配，正确平移）
run 3 b31   b 3 b31   reloc off none 31 12 $RATE
run 3 b33   b 3 b33   reloc off none 33 12 $RATE
# 3) B 段两组注入（开标定，沿用原错误外参预设，与 A 段既有注入同口径）
run 2 bpitch5 b 2 bpitch5 reloc on pitch5 8 $RATE
run 2 byaw5   b 2 byaw5   reloc on yaw5   8 $RATE
# 4) B 段 SLAM 基线（米级误差，门控阈值放宽至 6 m，评估用 Umeyama 对齐）
run 2 bslam   b 2 bslam   slam off none 30 8 $RATE

echo "############ 队列结束 $(date +%H:%M:%S) ############"
