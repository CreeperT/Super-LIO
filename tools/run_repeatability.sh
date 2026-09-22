#!/usr/bin/env bash
# 重复性实验：同一配置重复运行 n 次，逐次落盘。
# 与 run_pose_dc_experiments.sh 的差别：发散运行**不重试、不丢弃**，作为数据点保留，
# 以便在重复性表中披露真实的收敛率（重试包装器 run_with_retry.sh 会把发散次数吞掉，构成选择偏差）。
#
# 用法:
#   bash tools/run_repeatability.sh a 5     # A 段 ×5（自然失配 0.7°，关闭在线标定）
#   bash tools/run_repeatability.sh b 5     # B 段 ×5
#
# 输出: exp_repeat/<label>_r<i>/{traj.txt,pose_dc.csv,eval.log,launch.log,play.log}
#       exp_repeat/summary.csv（label,rep,converged,rmse_all,rmse_out,rmse_ret,ratio）
# 结束时自动还原 b2.yaml。
# 注意：ROS 的 setup.bash 引用了未绑定变量，不能用 set -u
set +u

WS=/home/creepert/Super-LIO
YAML=$WS/src/super_lio/config/b2.yaml
BAGS="$HOME/下载/rosbag"
OUT=$WS/exp_repeat
mkdir -p "$OUT"
cd "$WS" || exit 1
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

cp "$YAML" "$OUT/b2.yaml.bak"
restore() { cp "$OUT/b2.yaml.bak" "$YAML"; echo "[repeat] b2.yaml 已还原 $(date +%H:%M:%S)"; }
trap restore EXIT INT TERM

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
PY
}

cleanup() {  # 清残留：上一次的 bag play / 节点会污染本次运行（knn_valid→0 发散）
  pkill -f "ros2 bag play" 2>/dev/null
  pkill -f "super_lio/relocation_node" 2>/dev/null
  pkill -f "initial_pose_3d_marker" 2>/dev/null
  pkill -f "ros2 bag record" 2>/dev/null
  pkill -f "b2_relocation.launch.py" 2>/dev/null
  sleep 4
}

run_rep() {  # $1=label $2=bag $3=pitch $4=tx $5=ty $6=tz $7=init_pose $8=ref $9=rep
  local label=$1 bag=$2 pitch=$3 tx=$4 ty=$5 tz=$6 initp=$7 ref=$8 rep=$9
  local dir="$OUT/${label}_r${rep}"
  rm -rf "$dir"; mkdir -p "$dir"
  echo "=== [${label} r${rep}] 开始 $(date +%H:%M:%S)  bag=$(basename "$bag")  pitch=${pitch}°"

  cleanup
  set_pitch "$pitch" "$tx" "$ty" "$tz"
  set_yaml "lio.relocation.init_pose" "[$initp]"
  set_yaml "lio.eva.pose_dc_dump" "true"
  set_yaml "lio.calib.enable_online_calib" "false"
  : > "$WS/src/super_lio/logs/pose_dc_v2.csv"

  ros2 launch super_lio b2_relocation.launch.py rviz:=false > "$dir/launch.log" 2>&1 &
  local lpid=$!
  for _ in $(seq 1 60); do grep -q "Map size" "$dir/launch.log" && break; sleep 3; done
  sleep 25                      # 预热：地图加载完立即回放会丢失地图匹配
  echo "=== [${label} r${rep}] 预热完成 $(date +%H:%M:%S)"

  ros2 bag record -o "$dir/bag_odom" /lio/odom > "$dir/record.log" 2>&1 &
  local rpid=$!
  sleep 3
  timeout 1200 ros2 bag play "$bag" > "$dir/play.log" 2>&1
  sleep 5
  kill -INT $rpid 2>/dev/null; sleep 4; kill -9 $rpid 2>/dev/null
  kill -INT $lpid 2>/dev/null; sleep 6; kill -9 $lpid 2>/dev/null
  cleanup

  python3 "$WS/tools/export_traj.py" "$dir/bag_odom" "$dir/traj.txt" > "$dir/export.log" 2>&1
  cp "$WS/src/super_lio/logs/pose_dc_v2.csv" "$dir/pose_dc.csv" 2>/dev/null
  python3 "$WS/tools/eval_asymmetry.py" "$ref" "$dir/traj.txt" \
      --label "${label}_r${rep}" --trim_sec 30 > "$dir/eval.log" 2>&1

  python3 - "$dir" "$label" "$rep" "$OUT/summary.csv" <<'PY'
import pathlib, re, sys
d, label, rep, summ = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4]
txt = pathlib.Path(d, 'eval.log').read_text()
def num(pat):
    m = re.search(pat, txt)
    return m.group(1) if m else 'NA'
allm = num(r'全程\s+\d+\s+([\d.]+)')
out  = num(r'去程\s+\d+\s+([\d.]+)')
ret  = num(r'返程\s+\d+\s+([\d.]+)')
rat  = num(r'不对称率\(max\)\s+([\d.]+)')
ok = '1' if allm != 'NA' and float(allm) < 1.0 else '0'   # 与 run_with_retry.sh 同判据
p = pathlib.Path(summ)
if not p.exists():
    p.write_text('label,rep,converged,rmse_all,rmse_out,rmse_ret,ratio\n')
p.open('a').write(f'{label},{rep},{ok},{allm},{out},{ret},{rat}\n')
print(f'=== [{label} r{rep}] 收敛={ok}  全程={allm}  去程={out}  返程={ret}  比率={rat}')
PY
}

BAG_A="$BAGS/5#GJD"; INIT_A="50.716, 33.496, 44.762, 0.0, 0.0, 103.21"; REF_A="traj/reloc_traj.txt"
BAG_B="$BAGS/4#PSD"; INIT_B="90.655, -216.845, -7.111, 0.0, 0.0, -91.3"; REF_B="traj/reloc_traj_4#PSD.txt"

seg=$1; n=${2:-5}
for i in $(seq 1 "$n"); do
  case "$seg" in
    a) run_rep a30 "$BAG_A" 30 -0.3 0.0  0.15 "$INIT_A" "$REF_A" "$i" ;;
    b) run_rep b30 "$BAG_B" 30 -0.3 0.02 0.20 "$INIT_B" "$REF_B" "$i" ;;
    *) echo "用法: bash tools/run_repeatability.sh {a|b} [n]"; exit 1 ;;
  esac
done
echo "=== [$seg] $n 次全部完成 $(date +%H:%M:%S)"
