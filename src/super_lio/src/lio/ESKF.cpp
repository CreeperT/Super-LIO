#include "lio/ESKF.h"
#include "basic/logs.h"

using namespace BASIC;

namespace LI2Sup{


/// [1] SEfB: P195：（7.37a） P203: (7.76a) (7.77a)  P220: Table 7-2 
/// [2] VSLAM14: P71: (4.26)  P73: (4.32)
/// [3] Proportional Derivative (PD) Control on the Euclidean Group.  A_matrix
/// [1] = [2] <===> [3]
static inline M3d RightJacobianSO3(const V3& ang_vel, const scalar& dt){
  V3d ang_vel_d = ang_vel.template cast<double>();
  scalar ang_vel_norm = ang_vel_d.norm();
  if (ang_vel_norm < 1e-8){   /// noise? too small
    return M3d::Identity();
  }else{
    V3d r_axis = ang_vel_d / ang_vel_norm;
    double r_ang = ang_vel_norm * dt;
    M3d K;
    K << 0.0, -r_axis[2], r_axis[1], r_axis[2], 0.0, -r_axis[0], -r_axis[1], r_axis[0], 0.0;
    double a = (1 - std::cos(r_ang)) / r_ang;
    double b = 1 - std::sin(r_ang) / r_ang;
    return M3d::Identity() - a * K + b * K * K;   // J_r(phi) = J_l(-phi)
  }
}


/// left jacobian
M3d A_matrix(const V3 & v){
  M3d res;
  double squaredNorm = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
	double norm = std::sqrt(squaredNorm);
	if(norm < 1e-6){
		res = M3d::Identity();
	}
	else{
    M3d K;
    K << 0.0, -v[2], v[1], v[2], 0.0, -v[0], -v[1], v[0], 0.0;
		res = M3d::Identity() + (1 - std::cos(norm)) / squaredNorm * K 
        + (1 - std::sin(norm) / norm) / squaredNorm * K * K;
	}
  return res;
}


void ESKF::SetInitialConditions(Options options, const V3& init_bg,
                                const V3& init_ba, const float imu_scale,
                                const V3& gravity)
{
  BuildNoise(options);
  options_ = options;
  bg_ = init_bg;
  ba_ = init_ba;
  g_ = gravity;
  imu_scale_ = imu_scale;

  P_.setIdentity();
  P_ *= 1e-4;
  P_.template block<3, 3>(0, 0) = 0.1 * M_PI / 180.0 * M3::Identity();   // r
  P_.template block<3, 3>(12, 12) = 1e-2 * M3::Identity();                // ba: σ=0.1 m/s²

  // ── Phase 1: calibration initial uncertainty ──
  // Extrinsic rotation: 1° (generous initial guess from manual measurement)
  P_.template block<3, 3>(18, 18) = std::pow(3.0 * M_PI / 180.0, 2) * M3::Identity();  // ext1 rot
  P_.template block<3, 3>(21, 21) = std::pow(0.10, 2) * M3::Identity();                // ext1 trans: 10cm
  P_.template block<3, 3>(24, 24) = std::pow(3.0 * M_PI / 180.0, 2) * M3::Identity();  // ext2 rot
  P_.template block<3, 3>(27, 27) = std::pow(0.10, 2) * M3::Identity();                // ext2 trans: 10cm
  // Time offset: 5ms uncertainty
  P_(30, 30) = std::pow(5e-3, 2);
  P_(31, 31) = std::pow(5e-3, 2);

  // Init calibration nominal from globals
  T_L1_IMU_ = g_lidar_imu;
  T_L2_IMU_ = g_lidar_imu2;
  delta_t1_ = 0.0;  // ESKF δt is residual on top of ROSWrapper static offset
  delta_t2_ = 0.0;  // (static offsets already applied to start_time in ROSWrapper)
}


void ESKF::SetX(const SysState& x) {
  last_imu_time_ = x.timestamp;
  current_time_ = last_imu_time_;
  R_ = x.R;
  p_ = x.p;
  v_ = x.v;
  bg_ = x.bg;
  ba_ = x.ba;
  // calib: keep current values (set by SetInitialConditions or online estimation)
  fw_R_ = R_;
  fw_p_ = p_;
  fw_v_ = v_;
}


void ESKF::BuildNoise(const Options& options) {
  double et = options.gyro_var_;
  double ev = options.acce_var_;
  double eg = options.bias_gyro_var_;
  double ea = options.bias_acce_var_;

  double et2 = et;  // * et;
  double ev2 = ev;  // * ev;
  double eg2 = eg;  // * eg;
  double ea2 = ea;  // * ea;

  // set Q
  Q_.diagonal() << et2, et2, et2,    // ng: r
                   ev2, ev2, ev2,    // na: v  -> p
                   eg2, eg2, eg2,    // nbg: bg -> w -> r
                   ea2, ea2, ea2;    // nba: ba -> a -> v -> p
}

void ESKF::RemapDegeneracy(const M3& Q_trans, const M6& HTVH, STATE& dx) {
  if (!degen_opts_.enabled) {
    degen_detected_ = false;
    return;
  }

  // ── Compute current degeneracy metrics ──
  M3d HTVH_trans = HTVH.block<3,3>(3,3).cast<double>();
  Eigen::SelfAdjointEigenSolver<M3d> eig_H(HTVH_trans);
  V3d ev_H = eig_H.eigenvalues();
  double cond_H = (ev_H(2) > 1e-10) ? ev_H(2) / (ev_H(0) + 1e-10) : 0.0;

  bool currently_degenerate = (cond_H > degen_opts_.htvh_cond_thresh);

  // ── Recovery detection ──
  // When geometry improves (κ drops), count consecutive normal frames.
  // After N frames of good geometry, enter recovery: skip projection
  // and inflate P translation block to let observations pull the state back.
  static constexpr int RECOVERY_CONFIRM = 5;   // frames to confirm exit from degeneracy
  static constexpr int RECOVERY_FRAMES   = 10;  // frames of recovery boost

  if (!currently_degenerate) {
    recovery_confirm_count_++;
    if (recovery_confirm_count_ >= RECOVERY_CONFIRM &&
        recovery_remaining_ == 0 && was_degenerate_) {
      // Enter recovery mode
      recovery_remaining_ = RECOVERY_FRAMES;
      LOG(INFO) << GREEN << "[ESKF] Degeneracy recovery triggered — "
                << "geometry restored after " << recovery_confirm_count_
                << " normal frames, boosting observation for "
                << RECOVERY_FRAMES << " frames" << RESET;
      // Inflate P translation block to 1.0 m² — let observations dominate
      P_.block<3,3>(3,3) += M3::Identity() * scalar(1.0);
    }
  } else {
    recovery_confirm_count_ = 0;
    consecutive_degen_frames_++;
  }

  if (!currently_degenerate) {
    consecutive_degen_frames_ = 0;
    reset_map_requested_ = false;
  }

  // ── Map reset trigger: persistent degeneracy ──
  static constexpr int MAP_RESET_DEGEN_FRAMES = 100;
  if (consecutive_degen_frames_ > MAP_RESET_DEGEN_FRAMES && !reset_map_requested_) {
    reset_map_requested_ = true;
    LOG(WARNING) << "[ESKF] Persistent degeneracy " << consecutive_degen_frames_
                 << " frames — map reset requested";
  }

  if (recovery_remaining_ > 0) {
    recovery_remaining_--;
    degen_detected_ = false;
    was_degenerate_ = (recovery_remaining_ == 0) ? false : was_degenerate_;
    // During recovery: allow full dx (no projection), P already inflated
    return;
  }

  was_degenerate_ = currently_degenerate;
  degen_detected_ = false;

  // ── Check 1: posterior stddev ──
  Eigen::SelfAdjointEigenSolver<M3d> eig_Q(Q_trans.cast<double>());
  V3d ev_Q = eig_Q.eigenvalues();
  M3d V_f = M3d::Zero();
  int sigma_count = 0;
  for (int i = 0; i < 3; i++) {
    double stddev = std::sqrt(std::max(0.0, ev_Q(i)));
    if (stddev > degen_opts_.max_trans_stddev) {
      V_f += eig_Q.eigenvectors().col(i) * eig_Q.eigenvectors().col(i).transpose();
      sigma_count++;
    }
  }

  // ── Check 2: HTVH condition number ──
  // κ detects constraint IMBALANCE → warns that the weakest observation
  // direction may be unreliable. But κ alone is not enough: strong edge
  // features + calibration inflate λ_max, making κ large even when λ_min
  // provides adequate precision. Gate with an ABSOLUTE threshold on λ_min.
  //
  // Four-quadrant joint detection (κ + σ):
  //   κ小 σ小 → normal (no projection)
  //   κ大 σ小 → λ_min gate: project only if genuinely weak
  //   κ小 σ大 → σ projection (prior uncertainty)
  //   κ大 σ大 → σ + κ projection (both degraded)
  //
  // λ_min < 500: observation precision > 4.5cm → untrustworthy
  static constexpr double HTVH_MIN_EV_THRESH = 500.0;
  int cond_count = 0;
  if (currently_degenerate) {
    double ev_min = std::max(0.0, ev_H(0));
    if (ev_min < HTVH_MIN_EV_THRESH) {
      // Genuinely weak constraint — projection justified
      cond_count = 1;
      V3d weak_dir = eig_H.eigenvectors().col(0);
      V_f += weak_dir * weak_dir.transpose();
      LOG(WARNING) << "[ESKF] Degeneracy (κ): HTVH_trans κ=" << cond_H
                   << " > " << degen_opts_.htvh_cond_thresh
                   << " λ_min=" << ev_min << " < " << HTVH_MIN_EV_THRESH
                   << " | ev_H=[" << ev_H(0) << ", " << ev_H(1) << ", " << ev_H(2) << "]"
                   << " | suppressed weakest direction";
    } else {
      // Imbalanced but adequate — log only when verbose
      if (g_verbose_logging) {
        static int kappa_info_count = 0;
        if (++kappa_info_count <= 3 || kappa_info_count % 200 == 0) {
          LOG(INFO) << "[ESKF] HTVH κ=" << cond_H
                    << " > " << degen_opts_.htvh_cond_thresh
                    << " λ_min=" << ev_min << " ≥ " << HTVH_MIN_EV_THRESH
                    << " | adequate, no projection";
        }
      }
    }
  }

  // ── Apply projection (σ + gated-κ) ──
  if (sigma_count > 0 || cond_count > 0) {
    degen_detected_ = true;

    Eigen::SelfAdjointEigenSolver<M3d> eig_F(V_f);
    V3d ev_F = eig_F.eigenvalues();
    M3d V_f_clamped = M3d::Zero();
    for (int i = 0; i < 3; i++) {
      double clamped = std::min(ev_F(i), 1.0);
      V_f_clamped += clamped * eig_F.eigenvectors().col(i) * eig_F.eigenvectors().col(i).transpose();
    }

    V3d dx_trans = dx.segment<3>(3).cast<double>();
    dx_trans = (M3d::Identity() - V_f_clamped) * dx_trans;
    dx.segment<3>(3) = dx_trans.cast<scalar>();

    if (sigma_count > 0) {
      LOG(WARNING) << "[ESKF] Degeneracy (σ): " << sigma_count
                   << "/3 dims, σ=[" << std::sqrt(std::max(0.0, ev_Q(0)))
                   << ", " << std::sqrt(std::max(0.0, ev_Q(1)))
                   << ", " << std::sqrt(std::max(0.0, ev_Q(2))) << "]m"
                   << " > " << degen_opts_.max_trans_stddev << "m";
    }
  }
}


void ESKF::Update() {
  R_ = R_ * SO3::Exp(dx_.template block<3, 1>(0, 0));
  p_ += dx_.template block<3, 1>(3, 0);
  v_ += dx_.template block<3, 1>(6, 0);

  // Freeze bg/ba during degeneracy — unobservable directions only contain noise
  if (!degen_detected_) {
    bg_ += dx_.template block<3, 1>(9, 0);
    ba_ += dx_.template block<3, 1>(12, 0);
  }

  g_ += dx_.template block<3, 1>(15, 0);
  g_ = g_gravity_norm * (g_.normalized());

  // Calibration update is handled separately in UpdateObserve (block-diagonal)

  fw_R_ = R_;
  fw_p_ = p_;
  fw_v_ = v_;
  forward_time_ = current_obs_time_;
}


bool ESKF::Predict(const IMUData& imu, DynamicState& state_imu, DynamicState& state_robot){
  if(!init_) {
    return false;
  }

  /// first time predict
  if(forward_time_ < 0){
    forward_time_ = imu.secs;
    forward_last_imu_ = imu;
    return false;
  }

  double dt = imu.secs - forward_time_;

  if(dt < 0 || dt > 0.2){
    return false;
  }

  V3 acc = 0.5 * (imu.acc + forward_last_imu_.acc);
  acc = imu_scale_ * acc;
  acc = acc - ba_;
  
  V3 gyr = 0.5 * (imu.gyr + forward_last_imu_.gyr) - bg_;
  V3 new_p = fw_p_ + fw_v_ * dt + 0.5 * (fw_R_.R() * acc) * dt * dt + 0.5 * g_ * dt * dt;
  V3 new_v = fw_v_ + fw_R_.R() * acc * dt + g_ * dt;
  SO3 new_R = fw_R_ * SO3::Exp(gyr , dt);

  fw_R_ = new_R;
  fw_v_ = new_v;
  fw_p_ = new_p;

  state_imu.time = imu.secs;
  state_imu.R = fw_R_.R_;
  state_imu.p = fw_p_;
  state_imu.v = fw_v_;
  state_imu.w = gyr;
  state_imu.a = acc;
  forward_time_ = imu.secs;
  forward_last_imu_ = imu;

  new_R.R_ = fw_R_.R_ * g_odom_robo.R_;
  new_p = fw_R_.R_ * ( - g_odom_robo.R_ * g_odom_robo.t_) + fw_p_;
  state_robot.time = imu.secs;
  state_robot.R = new_R.R_;
  state_robot.p = new_p;
  // todo: v, w, a

  return true;
}


bool ESKF::Predict(const IMUData& imu) {

  if(last_imu_time_ < 0){
    last_imu_time_ = imu.secs;
    last_imu_ = imu;
    return false;
  }

  if(imu.secs <= last_obs_time_){
    last_imu_time_ = imu.secs;
    last_imu_ = imu;
    return false;
  }
  
  current_time_ = imu.secs;

  double dt;
  if(last_imu_time_ < last_obs_time_){
    dt = imu.secs - last_obs_time_;
  }else if (imu.secs > current_obs_time_){
    dt = current_obs_time_ - last_imu_time_;
    current_time_ = current_obs_time_;
  }else{
    dt = imu.secs - last_imu_time_;
  }

  V3 acc = 0.5 * (imu.acc + last_imu_.acc);
  acc = imu_scale_ * acc;
  acc = acc - ba_;
  body_omega_ = 0.5 * (imu.gyr + last_imu_.gyr) - bg_;
  M3 Jr_dt = (dt * RightJacobianSO3(body_omega_, dt)).cast<scalar>();   // J_l(-phi) = J_r(phi)

  M3 R_m3 = R_.R_;
  M3 R_dt = R_m3 * dt;

  F_X f_x = F_X::Identity();
  f_x.template block<3, 3>(0, 0) = SO3::Exp(-body_omega_, dt).R_;
  f_x.template block<3, 3>(0, 9) = - Jr_dt;
  f_x.template block<3, 3>(3, 6) = M3::Identity() * dt;
  f_x.template block<3, 3>(6, 0) = - R_m3 * SO3::hat(acc) * dt;
  f_x.template block<3, 3>(6, 12) = - R_dt;
  f_x.template block<3, 3>(6, 15) = M3::Identity() * dt;
  

  F_W f_w = F_W::Zero();
  f_w.template block<3, 3>(0, 0) = - Jr_dt;
  f_w.template block<3, 3>(6, 3) = - R_dt;                 // v -> na
  f_w.template block<3, 3>(9, 6) = M3::Identity() * dt;    // ba
  f_w.template block<3, 3>(12, 9) = M3::Identity() * dt;   // bg

  P_ = f_x * P_ * f_x.transpose() + f_w * Q_ * f_w.transpose();

  // ── Phase 1: calibration random walk noise ──
  if (calib_enabled_) {
    double dts = std::max(0.0, dt);
    scalar ext_rot_rw   = scalar(g_calib_ext_rot_rw   * std::sqrt(dts));
    scalar ext_trans_rw = scalar(g_calib_ext_trans_rw * std::sqrt(dts));
    scalar dt_rw        = scalar(g_calib_dt_rw        * std::sqrt(dts));
    for (int i = 0; i < 3; i++) {
      P_(18+i, 18+i) += ext_rot_rw * ext_rot_rw;
      P_(21+i, 21+i) += ext_trans_rw * ext_trans_rw;
      P_(24+i, 24+i) += ext_rot_rw * ext_rot_rw;
      P_(27+i, 27+i) += ext_trans_rw * ext_trans_rw;
    }
    P_(30, 30) += dt_rw * dt_rw;
    P_(31, 31) += dt_rw * dt_rw;
  }

  // P lower bounds (configurable via params)
  if (g_p_lower_bound_enabled) {
    scalar p_min_pred_trans = scalar(g_p_lower_bound_trans * g_p_lower_bound_trans);
    scalar p_min_pred_rot   = scalar(3.0e-6);
    for (int i = 3; i < 6; i++) {
      if (P_(i, i) < p_min_pred_trans) P_(i, i) = p_min_pred_trans;
    }
    for (int i = 0; i < 3; i++) {
      if (P_(i, i) < p_min_pred_rot) P_(i, i) = p_min_pred_rot;
    }
  }

  global_acc_ = R_.R() * acc + g_;
  p_ = p_ + v_ * dt + 0.5 * global_acc_ * dt * dt;
  v_ = v_ + global_acc_ * dt;
  R_ = R_ * SO3::Exp(body_omega_, dt);

  last_imu_time_ = imu.secs;
  last_imu_ = imu;
  return true;
}


const int STATE_DIM = ESKF::STATE_DIM_;  // keep for backward compat in RemapDegeneracy
bool ESKF::UpdateObserve(ESKF::ObsFunc obs) {
  SO3 R_0 = R_;

  M6  HTVH_pose;
  V6  HTVr_pose;
  OBS_MAT HTVH_calib;
  OBS_VEC HTVr_calib;

  COV Pk;
  NAV_MAT Qk_nav;

  need_converge_ = false;
  for (int iter = 0; iter < options_.num_iterations_; ++iter) {
    if(iter > 2){
      need_converge_ = true;
    }

    HTVH_pose.setZero(); HTVr_pose.setZero();
    HTVH_calib.setZero(); HTVr_calib.setZero();

    obs(GetKFState(), HTVH_pose, HTVr_pose, HTVH_calib, HTVr_calib);

    // J_theta: error-state Jacobian for rotation re-parameterization
    Pk = P_;
    M3 J_theta = M3::Identity() - 0.5 * SO3::hat((R_0.inverse() * R_).log_vee());
    // Apply J_theta only to the rotation row (0) and column (0)
    for(int j = 0; j < CORE_DIM + EXTRIN_DIM; j+=3){
      Pk.block<3,3>(0, j).noalias() = J_theta * P_.block<3,3>(0, j);
    }
    for(int j = 0; j < CORE_DIM + EXTRIN_DIM; j+=3){
      Pk.block<3,3>(j, 0) = Pk.block<3,3>(j, 0) * J_theta.transpose();
    }

    // ── Phase 1: 18×18 navigation solve (nav↔calib soft-decoupled) ──
    // Marginal nav prior information via Schur complement of P (covariance form):
    //   S = P_nn - P_nc·P_cc⁻¹·P_cn  (marginal nav covariance)
    //   P_nav⁻¹ = S⁻¹                (marginal nav information)
    // Avoids full 32×32 inverse: O(14³ + 18³) vs O(32³).
    Eigen::Matrix<scalar, NAV_DIM, NAV_DIM> P_nn = Pk.template block<NAV_DIM, NAV_DIM>(0, 0);
    Eigen::Matrix<scalar, NAV_DIM, CALIB_DIM> P_nc = Pk.template block<NAV_DIM, CALIB_DIM>(0, CORE_DIM);
    Eigen::Matrix<scalar, CALIB_DIM, CALIB_DIM> P_cc = Pk.template block<CALIB_DIM, CALIB_DIM>(CORE_DIM, CORE_DIM);
    Eigen::Matrix<scalar, CALIB_DIM, NAV_DIM> P_cn = Pk.template block<CALIB_DIM, NAV_DIM>(CORE_DIM, 0);

    NAV_MAT P_nav_marg = P_nn - P_nc * P_cc.ldlt().solve(P_cn);
    NAV_MAT Pk_inv_nav = P_nav_marg.inverse();

    // Add pose observation information to nav block
    Pk_inv_nav.template block<6,6>(0,0) += HTVH_pose;

    // RHS: only pose residual (6D), zeros for v/bg/ba/g
    NAV_VEC error_dx_nav = NAV_VEC::Zero();
    error_dx_nav.template block<6,1>(0, 0) = HTVr_pose;

    // Solve 18×18 navigation subsystem
    Qk_nav = Pk_inv_nav.inverse();
    NAV_VEC dx_nav = Qk_nav * error_dx_nav;

    // ── 18×18 vs 32×32 equivalence verification (paper support) ──
    if (g_decouple_verify) {
      static int cmp_count = 0;
      if (++cmp_count <= 10 || cmp_count % 500 == 0) {
        COV Pk_inv_full = Pk.inverse();
        Pk_inv_full.block<6,6>(0,0) += HTVH_pose;
        STATE err_full = STATE::Zero();
        err_full.block<6,1>(0,0) = HTVr_pose;
        COV Qk_full = Pk_inv_full.inverse();
        STATE dx_full = Qk_full * err_full;
        double diff_nav = (dx_nav - dx_full.head<18>()).norm();
        double diff_calib = dx_full.tail<14>().norm();
        LOG(INFO) << "[DecoupleVerify#" << cmp_count << "]"
                  << " ||dx_nav_diff||=" << diff_nav
                  << " ||dx_calib(full)||=" << diff_calib
                  << " ||dx_full||=" << dx_full.norm()
                  << " ||dx_dec||=" << dx_nav.norm()
                  << (diff_nav > 1e-3 ? " *** MISMATCH ***" : "");
      }
    }

    // Map nav correction to full state vector
    dx_.setZero();
    dx_.template block<18,1>(0, 0) = dx_nav;

    // ── DEBUG: solver health + Δx longitudinal tracking ──
    {
      static int solver_diag = 0;
      static double cum_dx_long = 0.0;
      solver_diag++;
      // Condition number of the 18×18 navigation IEKF system
      Eigen::JacobiSVD<NAV_MAT> svd(Pk_inv_nav);
      double cond_Pkinv = svd.singularValues()(0)
                        / (svd.singularValues()(svd.singularValues().size()-1) + 1e-30);
      // Eigenvalues of the 3×3 translation block of HTVH
      Eigen::SelfAdjointEigenSolver<M3d> eig_HTVH(HTVH_pose.block<3,3>(3,3).cast<double>());
      V3d ev_H = eig_HTVH.eigenvalues();
      // Longitudinal Δx — use eigenvector of weakest HTVH direction
      V3d weak_dir = eig_HTVH.eigenvectors().col(0);
      double dx_long = std::abs(dx_nav.template segment<3>(3).cast<double>().dot(weak_dir));
      cum_dx_long += dx_long;

      if (solver_diag <= 5 || solver_diag % 200 == 0) {
        LOG(INFO) << "[SolverDiag#" << solver_diag << "]"
                  << " cond(P⁻¹+H)=" << cond_Pkinv
                  << " ev_HTVH=[" << ev_H(0) << " " << ev_H(1) << " " << ev_H(2) << "]"
                  << " dx_long=" << dx_long << " cum_dx_long=" << cum_dx_long
                  << " P_lb=" << (g_p_lower_bound_enabled ? "ON" : "OFF");
      }
    }

    // solution remapping for degeneracy (uses navigation posterior translation covariance)
    RemapDegeneracy(Qk_nav.template block<3, 3>(3, 3), HTVH_pose, dx_);

    // update norm state (navigation only; calibration dimensions are zero in dx_)
    Update();

    if (dx_.lpNorm<Eigen::Infinity>() < options_.quit_eps_ && iter > 0) {
      break;
    }
  }

  // ── P update: nav block via information form (SPD guaranteed), cross via standard form ──
  // Information form: P_nav_post = (P_nav_prior⁻¹ + HᵀWH)⁻¹ = Qk_nav  — strictly SPD.
  // Cross-covariance: standard form propagation (always zero since P_nc ≡ 0, but
  // structurally correct for the general case).
  P_.template block<18,18>(0,0) = Qk_nav;

  NAV_MAT M_nav = NAV_MAT::Zero();
  M_nav.template block<6,6>(0,0) = HTVH_pose;
  NAV_MAT I_KH_nav = NAV_MAT::Identity() - Qk_nav * M_nav;
  P_.template block<18,14>(0,18).noalias() =
      I_KH_nav * Pk.template block<18,14>(0,18);
  P_.template block<14,18>(18,0) =
      P_.template block<18,14>(0,18).transpose();

  // Calibration block P_(18:31,18:31) unchanged — updated in Phase 2

  // ── Phase 1: separate calibration update (block-diagonal, after pose converged) ──
  if (calib_enabled_) {
    // Extract calibration sub-blocks from P
    Eigen::Matrix<scalar, 14, 14> P_calib = P_.block<14,14>(18, 18);
    Eigen::Matrix<scalar, 14, 14> S_calib = P_calib.inverse();
    S_calib += HTVH_calib.block<14,14>(6, 6).cast<scalar>();

    Eigen::Matrix<scalar, 14, 1> dx_calib =
        S_calib.ldlt().solve(HTVr_calib.template block<14,1>(6, 0).cast<scalar>());

    // Un-normalise rotation corrections: dθ_true = dθ_norm / s0
    if (calib_s0_ > 1.0) {
      scalar inv_s0 = scalar(1.0 / calib_s0_);
      dx_calib.template segment<3>(0) *= inv_s0;  // ext1 rot
      dx_calib.template segment<3>(6) *= inv_s0;  // ext2 rot
    }

    // Apply calibration update
    V6 dxi1;
    dxi1 << dx_calib.template segment<3>(0), dx_calib.template segment<3>(3);
    T_L1_IMU_.updateRhs(dxi1);

    V6 dxi2;
    dxi2 << dx_calib.template segment<3>(6), dx_calib.template segment<3>(9);
    T_L2_IMU_.updateRhs(dxi2);

    delta_t1_ += dx_calib(12);
    delta_t2_ += dx_calib(13);

    // Debug: print calibration correction every frame
    static int calib_dbg = 0;
    if (++calib_dbg <= 5 || calib_dbg % 200 == 0) {
      scalar deg1 = dx_calib.template segment<3>(0).norm() * 180.0 / M_PI;
      scalar deg2 = dx_calib.template segment<3>(6).norm() * 180.0 / M_PI;
      double ddt1 = dx_calib(12);  // time offset correction (seconds)
      double ddt2 = dx_calib(13);
      LOG(INFO) << "[CalibUpdate#" << calib_dbg
                << "] |dθ1|=" << deg1 << "° |dt1|=" << dx_calib.template segment<3>(3).norm() << "m"
                << " |dθ2|=" << deg2 << "° |dt2|=" << dx_calib.template segment<3>(9).norm() << "m"
                << " |dδt1|=" << (ddt1 * 1000.0) << "ms"
                << " |dδt2|=" << (ddt2 * 1000.0) << "ms"
                << " s0=" << calib_s0_;
    }

    // Update calibration covariance (standard form)
    Eigen::Matrix<scalar, 14, 14> K_calib = S_calib.ldlt().solve(
        Eigen::Matrix<scalar, 14, 14>::Identity());
    P_.block<14,14>(18, 18) = (Eigen::Matrix<scalar, 14, 14>::Identity()
                                - K_calib * HTVH_calib.block<14,14>(6, 6).cast<scalar>())
                               * P_calib;
  }

  Pk = P_;

  // project P with J_theta
  M3 J_theta = M3::Identity() - 0.5 * SO3::hat(dx_.template block<3, 1>(0, 0));
  for(int j = 0; j < CORE_DIM + EXTRIN_DIM; j+=3){
    Pk.block<3,3>(0, j).noalias() = J_theta * P_.block<3,3>(0, j);
  }
  for(int j = 0; j < CORE_DIM + EXTRIN_DIM; j+=3){
    Pk.block<3,3>(j, 0) = Pk.block<3,3>(j, 0) * J_theta.transpose();
  }

  P_ = Pk;

  // P lower bounds (configurable via params)
  if (g_verbose_logging) {
    scalar cur_trans_sigma = scalar(std::sqrt(std::max(scalar(0),
        (P_(3,3) + P_(4,4) + P_(5,5)) / scalar(3))));
    static scalar min_trans_sigma_seen = cur_trans_sigma;
    if (cur_trans_sigma < min_trans_sigma_seen) {
      min_trans_sigma_seen = cur_trans_sigma;
      LOG(INFO) << "[P_lb] new min σ_trans=" << min_trans_sigma_seen
                << "m (lb=" << g_p_lower_bound_trans << "m, "
                << (g_p_lower_bound_enabled ? "ON" : "OFF") << ")";
    }
  }
  if (g_p_lower_bound_enabled) {
    scalar p_min_trans = scalar(g_p_lower_bound_trans * g_p_lower_bound_trans);
    scalar p_min_rot   = scalar(3.0e-6);
    for (int i = 3; i < 6; i++) {
      if (P_(i, i) < p_min_trans) P_(i, i) = p_min_trans;
    }
    for (int i = 0; i < 3; i++) {
      if (P_(i, i) < p_min_rot) P_(i, i) = p_min_rot;
    }
  }

  dx_.setZero();

  last_obs_time_ = current_obs_time_;

  // ── Map-conflict detection ──
  // |HTVr| spikes when revisiting areas with drifted map points.
  {
    static int conflict_count = 0;
    float r_norm = HTVr_pose.norm();
    if (r_norm > 5000) {
      conflict_count++;
      if (conflict_count > 20 && !reset_map_requested_) {
        reset_map_requested_ = true;
        LOG(WARNING) << "[ESKF] Map conflict detected — |HTVr|=" << r_norm
                     << " for " << conflict_count << " frames";
      }
    } else {
      conflict_count = 0;
    }
  }

  return true;
}

}