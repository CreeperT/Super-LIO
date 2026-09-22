#!/usr/bin/env bash
# 自动复跑「关标定 + 剂量」实验，采集 pose_dc_v2.csv 与轨迹，用于式(20) 的定量验证。
#
# 用法:
#   bash tools/run_pose_dc_experiments.sh b30          # 只跑 B 段 30°（冒烟验证）
#   bash tools/run_pose_dc_experiments.sh all          # 全部四档
#   bash tools/run_pose_dc_experiments.sh a30 b31 b33  # 任意组合
#
# 每档流程: 改配置 → 清空 CSV → 启动节点 → 回放 bag → 录 /lio/odom → 停 → 导出轨迹 → 评估
# 结束时自动还原 b2.yaml 备份。
# 注意：ROS 的 setup.bash 引用了未绑定变量，不能用 set -u
set +u

WS=/home/creepert/Super-LIO
YAML=$WS/src/super_lio/config/b2.yaml
BAGS="$HOME/下载/rosbag"
OUT=$WS/exp_pose_dc
mkdir -p "$OUT"
cd "$WS" || exit 1
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

cp "$YAML" "$OUT/b2.yaml.bak"

# 关闭导出开关/标定开关/外参档位的统一入口
set_yaml() {  # $1=key $2=value（整行替换）
  python3 - "$1" "$2" <<'PY'
import pathlib, re, sys
key, val = sys.argv[1], sys.argv[2]
p = pathlib.Path('/home/creepert/Super-LIO/src/super_lio/config/b2.yaml')
L = p.read_text().splitlines(keepends=True)
for i, l in enumerate(L):
    if l.lstrip().startswith(key + ':'):
        L[i] = re.sub(r'^(\s*' + re.escape(key) + r':).*', r'\g<1> ' + val + '\n', l)
p.write_text(''.join(L))
PY
}

set_pitch() {  # $1=pitch(deg) $2=tx $3=ty $4=tz
  python3 - "$1" "$2" "$3" "$4" <<'PY'
import math, pathlib, sys
pitch, tx, ty, tz = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])
p = pathlib.Path('/home/creepert/Super-LIO/src/super_lio/config/b2.yaml')
L = p.read_text().splitlines(keepends=True)
idx = [k for k, l in enumerate(L) if l.lstrip().startswith('lio.lidar2.extrinsic_imu:')]
i = idx[-1]
c, s = math.cos(math.radians(pitch)), math.sin(math.radians(pitch))
L[i:i + 4] = [f"    lio.lidar2.extrinsic_imu: [{tx}, {ty}, {tz},\n",
              f"                                {c:.6f}, 0.0, {s:.6f},\n",
              f"                                0.0, 1.0, 0.0,\n",
              f"                              {-s:.6f}, 0.0, {c:.6f}]\n"]
p.write_text(''.join(L))
print(f'lidar2 extrinsics: pitch={pitch}° t=({tx},{ty},{tz})')
PY
}

run_one() {  # $1=label $2=bag $3=pitch $4=tx $5=ty $6=tz $7=init_pose $8=ref_traj
  local label=$1 bag=$2 pitch=$3 tx=$4 ty=$5 tz=$6 initp=$7 ref=$8
  echo "=================================================================="
  echo "[$label] 开始 $(date +%H:%M:%S)  bag=$(basename "$bag")  pitch=${pitch}°"
  set_pitch "$pitch" "$tx" "$ty" "$tz"
  set_yaml "lio.relocation.init_pose" "[$initp]"
  set_yaml "lio.eva.pose_dc_dump" "true"
  set_yaml "lio.calib.enable_online_calib" "false"
  : > "$WS/src/super_lio/logs/pose_dc_v2.csv"
  rm -rf "$OUT/bag_$label"

  ros2 launch super_lio b2_relocation.launch.py rviz:=false \
      > "$OUT/log_$label.launch.log" 2>&1 &
  local lpid=$!
  for _ in $(seq 1 60); do grep -q "Map size" "$OUT/log_$label.launch.log" && break; sleep 3; done
  # 预热 25 s：实测若地图加载完就立即回放（约 6 s），KF 会在运动开始后数秒丢失地图匹配
  # （knn_valid→0、位姿转纯 IMU 递推），预热后全程稳定。
  sleep 25
  echo "[$label] 地图加载完成并预热 $(date +%H:%M:%S)"

  ros2 bag record -o "$OUT/bag_$label" /lio/odom > "$OUT/log_$label.rec.log" 2>&1 &
  local rpid=$!
  sleep 3
  timeout 1200 ros2 bag play "$bag" > "$OUT/log_$label.play.log" 2>&1
  sleep 5
  kill -INT $rpid 2>/dev/null; sleep 4; kill -9 $rpid 2>/dev/null
  kill -INT $lpid 2>/dev/null; sleep 6; kill -9 $lpid 2>/dev/null
  pkill -f "super_lio/relocation_node" 2>/dev/null; pkill -f initial_pose_3d_marker 2>/dev/null
  sleep 3

  python3 "$WS/tools/export_traj.py" "$OUT/bag_$label" "$OUT/traj_$label.txt" \
      > "$OUT/log_$label.export.log" 2>&1
  echo "[$label] 轨迹行数: $(wc -l < "$OUT/traj_$label.txt" 2>/dev/null)  CSV 行数: $(wc -l < "$WS/src/super_lio/logs/pose_dc_v2.csv")  CSV 列数: $(awk -F, 'NR==1{print NF}' "$WS/src/super_lio/logs/pose_dc_v2.csv")（应为 116）"
  python3 "$WS/tools/eval_asymmetry.py" "$ref" "$OUT/traj_$label.txt" \
      --label "$label" --trim_sec 30 2>&1 | tail -8
  python3 "$WS/tools/analyze_pose_dc.py" --csv "$WS/src/super_lio/logs/pose_dc_v2.csv" \
      --plot --outdir "$OUT" 2>&1 | tail -12
  cp "$WS/src/super_lio/logs/pose_dc_v2.csv" "$OUT/pose_dc_$label.csv"
  echo "[$label] 结束 $(date +%H:%M:%S)"
}

BAG_A="$BAGS/5#GJD";     INIT_A="50.716, 33.496, 44.762, 0.0, 0.0, 103.21"; REF_A="traj/reloc_traj.txt"
BAG_B="$BAGS/4#PSD";     INIT_B="90.655, -216.845, -7.111, 0.0, 0.0, -91.3"; REF_B="traj/reloc_traj_4#PSD.txt"

sel="${*:-all}"
want() { [ "$sel" = "all" ] || [[ " $sel " == *" $1 "* ]]; }

want b30 && run_one b30 "$BAG_B" 30 -0.3 0.02 0.20 "$INIT_B" "$REF_B"
want a30 && run_one a30 "$BAG_A" 30 -0.3 0.0  0.15 "$INIT_A" "$REF_A"
want b31 && run_one b31 "$BAG_B" 31 -0.3 0.02 0.20 "$INIT_B" "$REF_B"
want b33 && run_one b33 "$BAG_B" 33 -0.3 0.02 0.20 "$INIT_B" "$REF_B"

cp "$OUT/b2.yaml.bak" "$YAML"
echo "全部完成，配置已还原。结果目录: $OUT"
