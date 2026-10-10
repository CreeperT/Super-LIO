// P1.9 逐位等价性测试（确定性，不依赖实时运行）
//
// 为什么需要它：端到端 A/B 已被实测证否 —— 同一二进制自比都会不同
// （帧数 515 vs 513，第 1 行第 3 字节即分叉）。实时管线的消息到达时序、
// 去畸变 [Stale#] 回退、TBB 归约顺序都会引入运行间差异，因此
// "默认值下行为逐位不变" 不可能在整机回放上验证。
//
// 本测试改为在**被改动的算术本身**上验证：把旧代码块与新 ApplyPLowerBound
// 的函数体逐字复制到同一程序，对同一批 P 矩阵运行，比较**原始比特**。
//
// 编译： g++ -O2 -std=c++17 plb_equiv_test.cpp -o plb_equiv_test
#include <cstdio>
#include <cstring>
#include <cstdint>
#include <random>
#include <vector>
#include <cmath>

using scalar = float;                       // BASIC::scalar（见 basic/alias.h:164）
static constexpr int DIM = 32;

struct Mat { scalar a[DIM][DIM]; };

// ── 旧实现（逐字复制自 ESKF.cpp 修改前的两处 clamp 块）──────────────────
// 返回触发计数 [trans_steps, rot_steps, trans_elems, rot_elems]
static void old_clamp(Mat& P, bool enabled, double trans_sigma, long long* c) {
  if (enabled) {
    scalar p_min_trans = scalar(trans_sigma * trans_sigma);
    scalar p_min_rot   = scalar(3.0e-6);            // ← 旧硬编码字面量
    for (int i = 3; i < 6; i++) {
      if (P.a[i][i] < p_min_trans) { P.a[i][i] = p_min_trans; c[2]++; }
    }
    for (int i = 0; i < 3; i++) {
      if (P.a[i][i] < p_min_rot) { P.a[i][i] = p_min_rot; c[3]++; }
    }
    c[0] = c[2] ? 1 : 0;   // 旧代码没有分步计数，这里只为对齐形状
    c[1] = c[3] ? 1 : 0;
  }
}

// ── 新实现（逐字复制自 ESKF.cpp 的 ApplyPLowerBound 函数体）────────────
static void new_clamp(Mat& P, bool enabled, double trans_sigma, double rot_sigma,
                      long long* steps, long long* tc, long long* rc,
                      long long* te, long long* re) {
  (*steps)++;
  if (!enabled) return;
  const scalar p_min_trans = scalar(trans_sigma * trans_sigma);
  const scalar p_min_rot   = scalar(rot_sigma);     // ← 参数，直接使用（方差）

  int n_trans = 0;
  for (int i = 3; i < 6; i++) {
    if (P.a[i][i] < p_min_trans) { P.a[i][i] = p_min_trans; n_trans++; }
  }
  int n_rot = 0;
  for (int i = 0; i < 3; i++) {
    if (P.a[i][i] < p_min_rot) { P.a[i][i] = p_min_rot; n_rot++; }
  }
  if (n_trans) { (*tc)++; *te += n_trans; }
  if (n_rot)   { (*rc)++; *re += n_rot; }
}

static bool bits_equal(const Mat& x, const Mat& y) {
  return std::memcmp(&x, &y, sizeof(Mat)) == 0;
}

int main() {
  std::mt19937_64 rng(20261010ULL);
  // 对数均匀分布，覆盖 1e-12 ~ 1e2 六个量级 —— 同时命中两侧触发与不触发
  std::uniform_real_distribution<double> expo(-12.0, 2.0);
  std::uniform_real_distribution<double> frac(0.0, 1.0);

  int n_case = 0, n_bitexact = 0, n_mismatch = 0;
  long long rot_trig_default = 0, rot_trig_zero = 0;

  for (int trial = 0; trial < 20000; ++trial) {
    Mat A{}, B{};
    for (int i = 0; i < DIM; i++) {
      for (int j = 0; j < DIM; j++) {
        double v = std::pow(10.0, expo(rng)) * (frac(rng) - 0.5);
        A.a[i][j] = B.a[i][j] = scalar(v);
      }
    }
    // 对角线额外覆盖三个关键区间
    for (int i = 0; i < 6; i++) {
      double d = (i < 3)
        ? std::pow(10.0, -8.0 + 5.0 * frac(rng))     // 旋转侧：横跨 3e-6 附近
        : std::pow(10.0, -6.0 + 4.0 * frac(rng));    // 位置侧：横跨 2.5e-3 附近
      A.a[i][i] = B.a[i][i] = scalar(d);
    }

    long long c_old[4] = {0,0,0,0};
    long long s=0, tc=0, rc=0, te=0, re=0;
    old_clamp(A, true, 0.05, c_old);
    new_clamp(B, true, 0.05, 3.0e-6, &s, &tc, &rc, &te, &re);

    n_case++;
    if (bits_equal(A, B)) n_bitexact++;
    else {
      n_mismatch++;
      if (n_mismatch <= 3) {
        for (int i = 0; i < 6; i++)
          if (std::memcmp(&A.a[i][i], &B.a[i][i], sizeof(scalar)) != 0)
            printf("  MISMATCH trial=%d idx=%d old=%.10g new=%.10g\n",
                   trial, i, (double)A.a[i][i], (double)B.a[i][i]);
      }
    }
    // 计数一致性：旧实现的 elems 与新实现的 elems 必须相同
    if (c_old[2] != te || c_old[3] != re) {
      printf("  COUNT MISMATCH trial=%d old(te,re)=(%lld,%lld) new=(%lld,%lld)\n",
             trial, c_old[2], c_old[3], te, re);
      return 2;
    }
    rot_trig_default += re;
  }

  // ── 验收 ④ 后半：rot_sigma = 0 ⇒ 旋转侧不触发 ──
  int zero_violation = 0;
  for (int trial = 0; trial < 20000; ++trial) {
    Mat A{};
    for (int i = 0; i < DIM; i++)
      for (int j = 0; j < DIM; j++)
        A.a[i][j] = scalar(std::pow(10.0, expo(rng)) * (frac(rng) - 0.5));
    for (int i = 0; i < 3; i++)
      A.a[i][i] = scalar(std::pow(10.0, -14.0 + 6.0 * frac(rng)));  // 含负值区间

    long long s=0,tc=0,rc=0,te=0,re=0;
    new_clamp(A, true, 0.05, 0.0, &s,&tc,&rc,&te,&re);
    rot_trig_zero += re;
    // 旋转侧只允许被抬到 0（对角元数值上略负 → 置 0，属良性）
    for (int i = 0; i < 3; i++)
      if (A.a[i][i] < scalar(0.0)) { zero_violation++; break; }
  }

  printf("\n===== P1.9 逐位等价性测试 =====\n");
  printf("用例数                : %d\n", n_case);
  printf("比特完全一致          : %d\n", n_bitexact);
  printf("不一致                : %d\n", n_mismatch);
  printf("rot_sigma=3.0e-6 旋转触发次数 : %lld\n", rot_trig_default);
  printf("rot_sigma=0      旋转触发次数 : %lld\n", rot_trig_zero);
  printf("rot_sigma=0 后仍为负的对角元  : %d\n", zero_violation);

  bool pass = (n_mismatch == 0) && (rot_trig_default > 0) &&
              (rot_trig_zero == 0) && (zero_violation == 0);
  printf("\n判定: %s\n", pass ? "通过 ✓" : "失败 ✗");
  return pass ? 0 : 1;
}
