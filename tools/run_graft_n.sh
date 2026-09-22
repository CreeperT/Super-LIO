#!/usr/bin/env bash
# 移植臂重复性评测：**预置移植外参** + 指定 ground_weight + 指定标定开关，跑到 n 轮有效为止。
#
# 为什么不复用 run_gated.sh：它的 `set_rear_extrinsic`（第 111 行起）**只接受 pitch 参数**，
# 内部构造 Ry(pitch)，结构上无法表达移植值里的 roll/yaw 与平移修正；且它每轮覆写外参。
# 本脚本直接按 12 个参数写入 `lio.lidar2.extrinsic_imu`。
#
# 有效判据 = 门控通过（gate_run.py：收敛/无退化抑制/跨度≥95%）**且** 覆盖率 ≥ min_coverage。
# 覆盖率不达标的轮次自动重跑，不计入 n。
#
# 用法:
#   bash tools/run_graft_n.sh <n_valid> [label] [calib] [ground_weight] [max_try] [min_coverage] [rw_zero]
#     n_valid      需要的有效轮数
#     label        输出目录前缀（默认 graft）
#     calib        on | off（默认 off）
#     ground_weight 地面通道权重（默认 1.0e7）
#     max_try      最大尝试次数（默认 n_valid*3）
#     min_coverage 覆盖率下限 %（默认 95）
#     rw_zero      yes → 三个标定随机游走置 0（默认 no）。注意：**不冻结**外参，
#                  初始协方差（rot 3° / trans 10 cm）仍在，前若干帧仍会移动。
set +u

WS=/home/creepert/Super-LIO
YAML=$WS/src/super_lio/config/b2.yaml
BAGS="$HOME/下载/rosbag"
OUT=$WS/exp_repeat
mkdir -p "$OUT"
cd "$WS" || exit 1
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

n_valid=${1:?需要 n_valid}; label=${2:-graft}; calib=${3:-off}
gweight=${4:-1.0e7}; max_try=${5:-$((n_valid * 3))}; min_cov=${6:-95}; rw_zero=${7:-no}
# 地面参考（可选，环境变量）：GRADE_AUTO=false GRADE_DEG=0.566 → 关闭自动捕获、用真值净坡度
GRADE_AUTO=${GRADE_AUTO:-}; GRADE_DEG=${GRADE_DEG:-}
BAG="$BAGS/5#GJD"; SPAN=816.8
# 移植值：取自 aslam_on1e7_r1 末态的**相对**外参（T_L1_IMU⁻¹⊗T_L2_IMU）与标称 ext1 复合
GRAFT_EXT="-0.29807, -0.01333, 0.20766, 0.868991, 0.026805, 0.494102, -0.029258, 0.999568, -0.002770, -0.493963, -0.012050, 0.869400"

cp "$YAML" "$OUT/b2.yaml.graftbak"
restore() { cp "$OUT/b2.yaml.graftbak" "$YAML"; echo "[graft] b2.yaml 已还原 $(date +%H:%M:%S)"; }
trap 'restore; exit 130' INT TERM
trap restore EXIT

set_yaml() {
  python3 - "$1" "$2" <<'PY'
import pathlib, re, sys
key, val = sys.argv[1], sys.argv[2]
p = pathlib.Path('/home/creepert/Super-LIO/src/super_lio/config/b2.yaml')
L = p.read_text().splitlines(keepends=True)
n = 0
for i, l in enumerate(L):
    if l.lstrip().startswith(key + ':'):
        L[i] = re.sub(r'^(\s*' + re.escape(key) + r':).*', r'\g<1> ' + val + '\n', l)
        n += 1
assert n == 1, f'set_yaml: 键 {key} 命中 {n} 次（应为 1）—— 旧版在此静默 no-op'
p.write_text(''.join(L))
PY
  [ $? -eq 0 ] || { echo "[graft] set_yaml 失败：键 $1 未被替换，中止本臂"; exit 1; }
}

apply_config() {
  set_yaml "lio.structure.ground_weight" "$gweight"
  set_yaml "lio.calib.enable_online_calib" "$([ "$calib" = on ] && echo true || echo false)"
  # 地面参考：默认保持自动捕获；指定 GRADE_AUTO=false 时改用 GRADE_DEG（真值净坡度 0.566°）
  if [ -n "$GRADE_AUTO" ]; then
    set_yaml "lio.structure.ground_grade_auto" "$GRADE_AUTO"
    set_yaml "lio.structure.ground_grade_deg" "${GRADE_DEG:-0.0}"
  fi
  python3 - "$GRAFT_EXT" <<'PY'
import pathlib, re, sys
key = 'lio.lidar2.extrinsic_imu'; val = sys.argv[1]
p = pathlib.Path('/home/creepert/Super-LIO/src/super_lio/config/b2.yaml')
t = p.read_text()
new = f"    {key}: [{val}]"
t2, n = re.subn(r'(?m)^    ' + re.escape(key) + r': \[[^\]]*\]', new, t, count=1)
assert n == 1, '未找到外参键'
p.write_text(t2)
PY
  [ $? -eq 0 ] || { echo "[graft] 外参写入失败：未匹配 lio.lidar2.extrinsic_imu"; exit 1; }
  [ "$rw_zero" = yes ] && {
    set_yaml "lio.calib.ext_rot_rw" "0.0"
    set_yaml "lio.calib.ext_trans_rw" "0.0"
    set_yaml "lio.calib.dt_rw" "0.0"
  }
  echo "[graft] 配置: calib=$calib weight=$gweight rw_zero=$rw_zero grade_auto=${GRADE_AUTO:-unchanged} grade_deg=${GRADE_DEG:--}"
  grep -n "extrinsic_imu: \[\|ground_weight\|enable_online_calib\|ground_grade\|_rw:" "$YAML" | grep -v "^\s*#" | head -9
}

cleanup() {
  pkill -f "[r]os2 bag play" 2>/dev/null
  pkill -f "[s]uper_lio_node" 2>/dev/null
  pkill -f "[r]os2 bag record" 2>/dev/null
  sleep 4
}

count_valid() {
  local n=0
  for d in "$OUT/${label}"_r*/; do
    [ -d "$d" ] || continue
    [ -f "$d/eval_ext.txt" ] || continue
    python3 - "$d" "$min_cov" <<'PY' && n=$((n + 1))
import re, sys, pathlib
d, mc = pathlib.Path(sys.argv[1]), float(sys.argv[2])
g = (d / 'gate.txt').read_text(errors='ignore') if (d / 'gate.txt').exists() else ''
c = re.search(r'覆盖率\s+([\d.]+)%', g)
ok = '有效 ✓' in g and c and float(c.group(1)) >= mc
sys.exit(0 if ok else 1)
PY
  done
  echo "$n"
}

next_attempt() { local k=1; while [ -d "$OUT/${label}_r${k}" ]; do k=$((k + 1)); done; echo "$k"; }

run_once() {
  local k=$1 dir="$OUT/${label}_r$1"
  mkdir -p "$dir"
  sha256sum "$YAML" > "$dir/config.sha256"   # 判据卡 0.2：每轮记录生效配置指纹（配合冻结基线比对）
  echo "=== [${label} r${k}] calib=$calib w=$gweight sha=$(cut -c1-12 "$dir/config.sha256") 开始 $(date +%H:%M:%S)"
  cleanup
  taskset -c 2-5 ros2 launch super_lio b2.launch.py rviz:=false > "$dir/launch.log" 2>&1 &
  local lpid=$!
  local ok=0
  for _ in $(seq 1 60); do grep -q "SuperLIO\]: initialized" "$dir/launch.log" && ok=1 && break; sleep 3; done
  [ "$ok" = 1 ] || { echo "  节点未就绪"; kill -9 $lpid 2>/dev/null; cleanup; return; }
  sleep 25
  taskset -c 10-11 ros2 bag record -o "$dir/bag_odom" /lio/odom > "$dir/record.log" 2>&1 &
  local rpid=$!
  sleep 3
  taskset -c 6-9 timeout 2400 ros2 bag play -r 1.0 "$BAG" > "$dir/play.log" 2>&1
  sleep 5
  kill -INT $rpid 2>/dev/null; sleep 4; kill -9 $rpid 2>/dev/null
  kill -INT $lpid 2>/dev/null; sleep 6; kill -9 $lpid 2>/dev/null
  cleanup

  python3 tools/export_traj.py "$dir/bag_odom" "$dir/traj.txt" > "$dir/export.log" 2>&1
  python3 tools/eval_asymmetry.py traj/reloc_traj.txt "$dir/traj.txt" --label "${label}_r${k}" --align > "$dir/eval.log" 2>&1
  python3 tools/gate_run.py "$dir" --span "$SPAN" --max-rmse 6.0 --csv > "$dir/gate.txt" 2>&1
  python3 tools/eval_ext.py "$dir" > "$dir/eval_ext.txt" 2>&1
  head -1 "$dir/gate.txt"
  sed -n '3,7p' "$dir/eval_ext.txt" | sed 's/^/  /'
}

# 表头由 eval_ext.py 统一产出（列清单的唯一出处，避免表头/数据列数漂移）
[ -f "$OUT/${label}_ext.csv" ] || python3 tools/eval_ext.py --header > "$OUT/${label}_ext.csv"

apply_config
valid=0; try=0
while [ "$valid" -lt "$n_valid" ] && [ "$try" -lt "$max_try" ]; do
  try=$((try + 1))
  k=$(next_attempt)
  run_once "$k"
  python3 tools/eval_ext.py "$OUT/${label}_r${k}" --csv >> "$OUT/${label}_ext.csv" 2>/dev/null
  valid=$(count_valid)
  echo "=== [${label}] 有效 $valid / $n_valid（已尝试 $try 次）"
done
echo "=== [${label}] 结束：有效 $valid / $n_valid，共尝试 $try 次 $(date +%H:%M:%S)"
echo "=== 扩展指标表 ==="
column -s, -t "$OUT/${label}_ext.csv" | tail -n +2
