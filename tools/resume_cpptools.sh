#!/usr/bin/env bash
# 队列结束后自动恢复被暂停的 cpptools（VS Code C++ 语言服务）。
# 暂停原因：它跑满一核并占 2.7GB，实测使 relocation_node 的处理速率从 15.6 Hz 掉到 8.5 Hz，
# 进而导致 odom 记录掉帧（覆盖率 63%）、复跑轮次被判无效。
set +u
while pgrep -f "[r]un_queue.sh" >/dev/null; do sleep 30; done
for p in $(pgrep -f "[c]pptools"); do
  kill -CONT "$p" 2>/dev/null && echo "$(date +%H:%M:%S) 已恢复 cpptools pid=$p"
done
