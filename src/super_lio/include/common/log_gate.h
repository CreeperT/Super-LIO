/**
 * @file log_gate.h
 * @brief 帧计数限频闸门 —— 让"每 N 帧打印一次"变成可配置量而不是写死的常量。
 *
 * 用法（每个日志点一个函数内 static 闸门）：
 *   static LogGate gate(5);                     // 前 5 次无条件放行（对齐原有手感）
 *   if (log_hit(gate, g_log_bias, g_log_bias_frames)) LOG(INFO) << "[Bias#" << gate.count() << ...;
 *
 * log_hit() 在 lio/params.h 里定义：它把 item_enabled 与总开关 g_log_enable 相与，
 * period<=0 表示该点不再打印。频率口径与"性能评估"一致：period=1 即每帧（eva 计时项）。
 */

#ifndef LOG_GATE_H_
#define LOG_GATE_H_

namespace LI2Sup {

class LogGate {
 public:
  explicit LogGate(long first_n = 0) : first_n_(first_n) {}

  /// 返回本次是否应打印。enabled=false 时恒 false 且不推进计数以外的任何状态。
  bool hit(bool enabled, int period) {
    if (!enabled) return false;
    ++n_;
    if (n_ <= first_n_) return true;
    if (period <= 0) return false;
    return (n_ % static_cast<long>(period)) == 0;
  }

  /// 已观察到的次数（打印时用作 "#N" 里的 N，保证编号连续可读）
  long count() const { return n_; }

 private:
  long n_ = 0;
  long first_n_ = 0;
};

}  // namespace LI2Sup

#endif  // LOG_GATE_H_
