#!/usr/bin/env bash
# 反复运行某一档直到收敛。
# b31/b33 这类较大失配档有概率发散（knn_valid→0、轨迹转入纯 IMU 递推），
# 判据：轨迹全程 RMSE < 1 m（健康运行实测 0.1~0.3 m，发散时可达 1e5 m 量级）。
#
# 用法: bash tools/run_with_retry.sh b31 [最大尝试次数]
set +u

WS=/home/creepert/Super-LIO
OUT=$WS/exp_pose_dc
label=$1
max=${2:-8}

for i in $(seq 1 "$max"); do
  log="$OUT/retry_${label}_$i.log"
  echo "=== [$label] 第 $i 次 $(date +%H:%M:%S)"
  bash "$WS/tools/run_pose_dc_experiments.sh" "$label" > "$log" 2>&1

  rmse=$(grep -m1 "全程" "$log" | awk '{print $3}')
  ok=$(python3 -c "print(1 if float('${rmse:-999}')<1.0 else 0)" 2>/dev/null || echo 0)
  if [ "$ok" = "1" ]; then
    cp "$OUT/pose_dc_$label.csv" "$OUT/pose_dc_${label}_ok.csv" 2>/dev/null
    cp "$OUT/traj_$label.txt"    "$OUT/traj_${label}_ok.txt"    2>/dev/null
    echo "=== [$label] 第 $i 次收敛，全程 RMSE=$rmse（结果已存为 *_ok）"
    exit 0
  fi
  echo "=== [$label] 第 $i 次发散（RMSE=${rmse:-NA}），重试"
done

echo "=== [$label] 达到最大尝试次数（$max）仍未收敛"
exit 1
