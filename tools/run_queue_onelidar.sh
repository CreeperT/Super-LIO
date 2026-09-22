#!/usr/bin/env bash
# 单雷达消融重采（A 段、1×、正确平移、关闭后雷达）
set +u
cd /home/creepert/Super-LIO || exit 1
echo "############ $(date +%H:%M:%S) 单雷达消融 ############"
bash tools/run_gated.sh a 3 aonelidar reloc off onelidar 30 10 1.0 || echo "!!! 未达目标"
echo "############ 结束 $(date +%H:%M:%S) ############"
