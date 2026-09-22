#!/usr/bin/env bash
# 带门控的复跑：反复运行同一配置，直到累计 n 次**有效**运行为止。
# 有效判据见 tools/gate_run.py（收敛 / 无初始化期退化抑制 / 覆盖率≥95% / 记录时长≥95%）。
# 无效运行不进入记录，另存 discarded 行；已存在的有效运行计入目标（可断点续跑）。
#
# 用法:
#   bash tools/run_gated.sh <seg> <n_valid> <label> [mode] [calib] [inj] [pitch] [max_try]
#     seg   = a | b
#     mode  = reloc（默认） | slam
#     calib = off（默认） | on
#     inj   = none（默认） | pitch5（后雷达 +5°） | yaw5（前雷达 +5°）
#     pitch = 后雷达设定角（注入时忽略），默认 30；剂量档传 31 / 33
#
# 外参口径（本脚本每轮把两个外参键**都**显式写死，避免上一轮残留）：
#   前雷达 lio.extrinsic.lidar_imu     : 正确值 / yaw+5° 预设（含 +5cm x、+5cm z）
#   后雷达 lio.lidar2.extrinsic_imu    : 正确平移 (-0.3,0,0.15) + 设定 pitch
#                                        pitch5 注入沿用原错误预设 (-0.3,0.02,0.20)+35°
# 例：
#   bash tools/run_gated.sh a 5 a30                       # A 关标定自然失配
#   bash tools/run_gated.sh b 5 b30 reloc off none 30     # B 关标定（正确平移）
#   bash tools/run_gated.sh b 3 b31 reloc off none 31     # B 剂量 31°
#   bash tools/run_gated.sh b 2 bslam reloc off none 30 slam
#   bash tools/run_gated.sh b 2 bpitch5 reloc on pitch5   # 开标定 + 后雷达注入
set +u

WS=/home/creepert/Super-LIO
YAML=$WS/src/super_lio/config/b2.yaml
BAGS="$HOME/下载/rosbag"
OUT=$WS/exp_repeat
VALID=$OUT/summary_valid.csv
DISC=$OUT/discarded.csv
mkdir -p "$OUT"
cd "$WS" || exit 1
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

seg=$1; n_target=$2; label=$3
mode=${4:-reloc}; calib=${5:-off}; inj=${6:-none}; pitch=${7:-30}; max_try=${8:-$((n_target * 4))}
rate=${9:-1.0}   # 回放速率：节点吞吐不足时降速（消息时间戳不变，轨迹时间轴与 1× 一致）
# 防呆：参数错位会让 while 的算术比较失败并静默空转（曾导致两组配置一次都没跑）
case "$max_try" in ''|*[!0-9]*) echo "!! max_try 必须为非负整数（收到 '$max_try'）——检查参数顺序"; exit 1 ;; esac
case "$rate" in ''|*[!0-9.]*) echo "!! rate 必须为数字（收到 '$rate'）"; exit 1 ;; esac

case "$seg" in
  a) BAG="$BAGS/5#GJD"; INIT="50.716, 33.496, 44.762, 0.0, 0.0, 103.21"
     REF="traj/reloc_traj.txt";      SPAN=816.8 ;;
  b) BAG="$BAGS/4#PSD"; INIT="90.655, -216.845, -7.111, 0.0, 0.0, -91.3"
     REF="traj/reloc_traj_4#PSD.txt"; SPAN=523.4 ;;
  *) echo "seg 只能是 a 或 b"; exit 1 ;;
esac

case "$mode" in slam) LAUNCH=b2.launch.py; ALIGN="--align"; MAX_RMSE=6.0; TRIM="" ;;       # 论文口径：SLAM 不剔除启动瞬态
               *)    LAUNCH=b2_relocation.launch.py; ALIGN=""; MAX_RMSE=1.0; TRIM="--trim_sec 30" ;; esac

# 注入组（±5°）沿用 b2.yaml 中的**原错误外参预设**：旋转 + 平移同时注入
# （平移误差收敛很快，论文未作讨论，但它是该错误组的一部分）。
# 基线/剂量档（inj=none）则一律使用**正确平移**，使"关闭在线标定"只体现自然失配。
case "$inj" in
  pitch5)   REAR_PITCH=35;     REAR_T="-0.3 0.02 0.20"; FRONT=ok;   LIDAR2=true ;;   # 与 A 段既有注入同口径
  yaw5)     REAR_PITCH=$pitch; REAR_T="-0.3 0.0 0.15";  FRONT=yaw5; LIDAR2=true ;;   # 前雷达注入，后雷达保正确值
  onelidar) REAR_PITCH=$pitch; REAR_T="-0.3 0.0 0.15";  FRONT=ok;   LIDAR2=false ;;  # 单雷达消融：关闭后雷达
  *)        REAR_PITCH=$pitch; REAR_T="-0.3 0.0 0.15";  FRONT=ok;   LIDAR2=true ;;
esac
# shellcheck disable=SC2086
set -- $REAR_T; REAR_TX=$1; REAR_TY=$2; REAR_TZ=$3

cp "$YAML" "$OUT/b2.yaml.bak"
restore() { cp "$OUT/b2.yaml.bak" "$YAML"; echo "[gated] b2.yaml 已还原 $(date +%H:%M:%S)"; }
# 收到中断信号必须**退出**：只还原配置而继续跑，会在"配置已还原、节点已被杀"的状态下白跑一整轮
trap 'restore; exit 130' INT TERM
trap restore EXIT

set_yaml() {
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

set_extrinsic() {   # $1=键名 $2..13=12 个数（多行数组整体替换，必须保留原缩进）
  python3 - "$@" <<'PY'
import pathlib, re, sys
key = sys.argv[1]
v = sys.argv[2:14]
assert len(v) == 12, '需要 12 个数值'
p = pathlib.Path('/home/creepert/Super-LIO/src/super_lio/config/b2.yaml')
txt = p.read_text()
pat = re.compile(r'(?m)^([ \t]*)' + re.escape(key) + r':[ \t]*\[[^\]]*\]')

def repl(m):
    ind = m.group(1)                 # 键的缩进（ros__parameters 层级）必须原样保留
    cont = ind + ' ' * 28            # 续行相对键再缩进
    return (f"{ind}{key}: [{v[0]}, {v[1]}, {v[2]},\n"
            f"{cont}{v[3]}, {v[4]}, {v[5]},\n"
            f"{cont}{v[6]}, {v[7]}, {v[8]},\n"
            f"{cont}{v[9]}, {v[10]}, {v[11]}]")

new, n = pat.subn(repl, txt, count=1)
assert n == 1, f'未找到外参键 {key}'
assert len(m.group(1)) > 0 if (m := pat.search(txt)) else True, '键前缩进为空，层级会丢'
p.write_text(new)
PY
}

set_rear_extrinsic() {  # $1=pitch(deg) $2..4 = tx ty tz
  python3 - "$1" "$2" "$3" "$4" <<'PY'
import math
pitch, tx, ty, tz = (float(a) for a in __import__('sys').argv[1:5])
c, s = math.cos(math.radians(pitch)), math.sin(math.radians(pitch))
print(f"{tx} {ty} {tz} {c:.6f} 0.0 {s:.6f} 0.0 1.0 0.0 {-s:.6f} 0.0 {c:.6f}")
PY
}

apply_extrinsics() {
  # 前雷达
  if [ "$FRONT" = "yaw5" ]; then
    set_extrinsic "lio.extrinsic.lidar_imu" 0.39218 0.0 0.22851 \
        0.996195 0.087156 0.0 -0.087156 0.996195 0.0 0.0 0.0 1.0
  else
    set_extrinsic "lio.extrinsic.lidar_imu" 0.34218 0.0 0.17851 \
        1.0 0.0 0.0 0.0 1.0 0.0 0.0 0.0 1.0
  fi
  # 后雷达（变量名避开 run_once 的局部变量 k，否则日志里的轮次号会被覆盖）
  read -r r0 r1 r2 r3 r4 r5 r6 r7 r8 r9 r10 r11 <<< "$(set_rear_extrinsic "$REAR_PITCH" "$REAR_TX" "$REAR_TY" "$REAR_TZ")"
  set_extrinsic "lio.lidar2.extrinsic_imu" "$r0" "$r1" "$r2" "$r3" "$r4" "$r5" "$r6" "$r7" "$r8" "$r9" "$r10" "$r11"
}

cleanup() {
  pkill -f "[r]os2 bag play" 2>/dev/null
  pkill -f "[r]elocation_node" 2>/dev/null
  pkill -f "[s]uper_lio_node" 2>/dev/null
  pkill -f "[i]nitial_pose_3d_marker" 2>/dev/null
  pkill -f "[r]os2 bag record" 2>/dev/null
  sleep 4
}

count_valid() {
  # 只统计**同一回放协议**的有效轮次：0.7× 与 1× 不可混用，速率不符者不计入目标
  local n=0 d rr=${rate%.0}
  for d in "$OUT/${label}"_r*/; do
    [ -d "$d" ] || continue
    grep -q "Set rate to ${rr}$" "$d/play.log" 2>/dev/null || continue
    python3 tools/gate_run.py "$d" --span "$SPAN" --max-rmse "$MAX_RMSE" 2>/dev/null \
      | head -1 | grep -q "有效 ✓" && n=$((n + 1))
  done
  echo "$n"
}

next_attempt() {
  local k=1
  while [ -d "$OUT/${label}_r${k}" ]; do k=$((k + 1)); done
  echo "$k"
}

run_once() {
  local k=$1 dir="$OUT/${label}_r$1"
  mkdir -p "$dir"
  echo "=== [${label} r${k}] $mode/calib-$calib/inj-$inj/pitch-${REAR_PITCH}° @${rate}x 开始 $(date +%H:%M:%S)"
  cleanup
  apply_extrinsics
  set_yaml "lio.relocation.init_pose" "[$INIT]"
  set_yaml "lio.lidar2.enable" "$LIDAR2"      # 单雷达消融时关闭后雷达，其余模式必须显式写回 true
  set_yaml "lio.eva.pose_dc_dump" "true"
  set_yaml "lio.calib.enable_online_calib" "$([ "$calib" = on ] && echo true || echo false)"
  : > "$WS/src/super_lio/logs/pose_dc_v2.csv"

  # 分核绑定：节点独占 2-5、回放独占 6-9、录制 10-11，避免与桌面程序（Chrome/Zotero/ToDesk）
  # 或彼此争抢同一物理核的超线程而拖慢实时处理（实测前雷达交付会从 10 Hz 掉到 8.2 Hz）。
  taskset -c 2-5 ros2 launch super_lio "$LAUNCH" rviz:=false > "$dir/launch.log" 2>&1 &
  local lpid=$!
  local loaded=0
  # 就绪标志按模式区分：Map size 只有重定位节点会打印，SLAM 节点打印的是 initialized
  local ready="Map size"
  [ "$mode" = slam ] && ready="SuperLIO\]: initialized"
  for _ in $(seq 1 60); do grep -q "$ready" "$dir/launch.log" && loaded=1 && break; sleep 3; done
  if [ "$loaded" != "1" ]; then     # params 解析失败/地图加载失败：立即放弃本轮，不白跑 14 分钟回放
    echo "=== [${label} r${k}] 节点未能加载地图，跳过回放"
    kill -INT $lpid 2>/dev/null; sleep 3; kill -9 $lpid 2>/dev/null; cleanup
    grep -m1 "Error at line\|Failed to parse\|Error:" "$dir/launch.log" >> "$dir/gate.txt" 2>/dev/null
    echo "$dir,0,NA,NA,NA,NA,NA,0" >> "$DISC"
    return
  fi
  sleep 25
  taskset -c 10-11 ros2 bag record -o "$dir/bag_odom" /lio/odom > "$dir/record.log" 2>&1 &
  local rpid=$!
  sleep 3
  # 回放期间轮询：重定位模式下若出现早期失配（丢地图匹配／退化抑制），立即中止本轮，
  # 把浪费从"整轮 11–17 分钟"压到"约 1.5 分钟"。SLAM 模式无地图匹配，不做此判定。
  if [ "$mode" = reloc ]; then
    taskset -c 6-9 timeout 2400 ros2 bag play -r "$rate" "$BAG" > "$dir/play.log" 2>&1 &
    local bp=$!
    while kill -0 "$bp" 2>/dev/null; do
      sleep 20
      local nz deg
      nz=$(grep -c "knn_valid=0" "$dir/launch.log" 2>/dev/null); nz=${nz:-0}
      deg=$(grep -c "suppressed weakest direction" "$dir/launch.log" 2>/dev/null); deg=${deg:-0}
      if [ "$nz" -gt 5 ] || [ "$deg" -gt 3 ]; then
        echo "=== [${label} r${k}] 早期失配（knn_valid=0 共 $nz 行 / 退化抑制 $deg 次），提前中止"
        pkill -f "[r]os2 bag play" 2>/dev/null
        break
      fi
    done
    wait "$bp" 2>/dev/null
  else
    taskset -c 6-9 timeout 2400 ros2 bag play -r "$rate" "$BAG" > "$dir/play.log" 2>&1
  fi
  echo "=== [${label} r${k}] 回放结束 $(date +%H:%M:%S)"
  sleep 5
  kill -INT $rpid 2>/dev/null; sleep 4; kill -9 $rpid 2>/dev/null
  kill -INT $lpid 2>/dev/null; sleep 6; kill -9 $lpid 2>/dev/null
  cleanup

  python3 "$WS/tools/export_traj.py" "$dir/bag_odom" "$dir/traj.txt" > "$dir/export.log" 2>&1
  cp "$WS/src/super_lio/logs/pose_dc_v2.csv" "$dir/pose_dc.csv" 2>/dev/null
  # shellcheck disable=SC2086
  python3 "$WS/tools/eval_asymmetry.py" "$REF" "$dir/traj.txt" \
      --label "${label}_r${k}" $TRIM $ALIGN > "$dir/eval.log" 2>&1

  python3 "$WS/tools/gate_run.py" "$dir" --span "$SPAN" --max-rmse "$MAX_RMSE" --csv > "$dir/gate.txt" 2>&1
  head -1 "$dir/gate.txt"
  if grep -q "有效 ✓" "$dir/gate.txt"; then
    tail -1 "$dir/gate.txt" >> "$VALID"; echo "=== [${label} r${k}] 计入有效运行"
  else
    tail -1 "$dir/gate.txt" >> "$DISC";  echo "=== [${label} r${k}] 判为无效，不计入记录"
  fi
}

[ -f "$VALID" ] || echo 'dir,valid,rmse_all,rmse_out,rmse_ret,ratio,deg,coverage' > "$VALID"
[ -f "$DISC" ]  || echo 'dir,valid,rmse_all,rmse_out,rmse_ret,ratio,deg,coverage' > "$DISC"

valid=$(count_valid)
echo "=== [${label}] 起始有效 $valid / 目标 $n_target（$mode, calib=$calib, inj=$inj, pitch=${REAR_PITCH}°, 平移=$REAR_TX,$REAR_TY,$REAR_TZ）"
try=0
while [ "$valid" -lt "$n_target" ] && [ "$try" -lt "$max_try" ]; do
  try=$((try + 1))
  run_once "$(next_attempt)"
  valid=$(count_valid)
  echo "=== [${label}] 进度 $valid / $n_target（已尝试 $try 次）"
done
echo "=== [${label}] 结束：有效 $valid / $n_target，共尝试 $try 次 $(date +%H:%M:%S)"
