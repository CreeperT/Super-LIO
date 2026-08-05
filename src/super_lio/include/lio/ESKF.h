#ifndef ESKF_HPP_
#define ESKF_HPP_

#include "basic/alias.h"
#include "basic/Manifold.h"
#include "common/ds.h"
#include "params.h"


namespace LI2Sup{

class ESKF {
public:
  using Ptr   = std::shared_ptr<ESKF>;

  static constexpr int CORE_DIM   = 18;  // R(3) p(3) v(3) bg(3) ba(3) g(3)
  static constexpr int EXTRIN_DIM = 12;  // T_ext1(6) T_ext2(6)
  static constexpr int TIME_DIM   = 2;   // dt1(1) dt2(1)
  static constexpr int STATE_DIM_ = CORE_DIM + EXTRIN_DIM + TIME_DIM;  // 32
  static constexpr int OBS_DIM    = 6 + EXTRIN_DIM + TIME_DIM;         // 20

  // Two-step decoupling: navigation (18×18) + calibration (14×14)
  static constexpr int NAV_DIM   = CORE_DIM;                       // 18
  static constexpr int CALIB_DIM = EXTRIN_DIM + TIME_DIM;          // 14

  using STATE = Eigen::Matrix<BASIC::scalar, STATE_DIM_, 1>;
  using STATE_DOF = Eigen::Matrix<BASIC::scalar, 17, 1>;
  using NOISE = Eigen::Matrix<BASIC::scalar, 12, 12>;       // unchanged: IMU noise only
  using COV   = Eigen::Matrix<BASIC::scalar, STATE_DIM_, STATE_DIM_>;
  using F_X   = Eigen::Matrix<BASIC::scalar, STATE_DIM_, STATE_DIM_>;
  using F_W   = Eigen::Matrix<BASIC::scalar, STATE_DIM_, 12>;

  using NAV_MAT = Eigen::Matrix<BASIC::scalar, NAV_DIM, NAV_DIM>;
  using NAV_VEC = Eigen::Matrix<BASIC::scalar, NAV_DIM, 1>;
  using CALIB_MAT = Eigen::Matrix<BASIC::scalar, CALIB_DIM, CALIB_DIM>;
  using CALIB_VEC = Eigen::Matrix<BASIC::scalar, CALIB_DIM, 1>;


  struct Options {
    Options(){}
    int num_iterations_ = 3;
    double quit_eps_ = 1e-6;

    double gyro_var_ = 1e-5;
    double acce_var_ = 1e-2;
    double bias_gyro_var_ = 1e-6;
    double bias_acce_var_ = 1e-4;
  };


  struct KFState{
    bool need_converge = true;
    BASIC::SE3  pose;
    BASIC::SE3  T_L1_IMU;   // LiDAR1→IMU extrinsic (nominal)
    BASIC::SE3  T_L2_IMU;   // LiDAR2→IMU extrinsic (nominal)
    double      dt1 = 0.0;  // LiDAR1 time offset (nominal)
    double      dt2 = 0.0;  // LiDAR2 time offset (nominal)
  };

  struct DegeneracyOptions {
    bool enabled = true;
    double max_trans_stddev = 0.5;   // posterior σ > this (meters) → degenerate
    double htvh_cond_thresh = 100.0; // HTVH trans block condition number > this → degenerate
  };


  ESKF(Options option = Options()) : options_(option) { BuildNoise(option); }

  ESKF(Options options, const BASIC::V3& init_bg, const BASIC::V3& init_ba, const BASIC::V3& gravity = BASIC::V3(0, 0, -9.8))
      : options_(options) {
    BuildNoise(options);
    bg_ = init_bg;
    ba_ = init_ba;
    g_ = gravity;
  }

  void SetInitialConditions(Options options, const BASIC::V3& init_bg, const BASIC::V3& init_ba, const float imu_scale = 1.0,
                            const BASIC::V3& gravity = BASIC::V3(0, 0, -9.8));

  bool Predict(const IMUData& imu);

  // Observation return: pose(6) + calib(EXTRIN_DIM+TIME_DIM) = OBS_DIM
  using OBS_MAT = Eigen::Matrix<BASIC::scalar, OBS_DIM, OBS_DIM>;
  using OBS_VEC = Eigen::Matrix<BASIC::scalar, OBS_DIM, 1>;
  using ObsFunc = std::function<void(const KFState& kf_state,
                                      BASIC::M6& HTVH_pose, BASIC::V6& HTVr_pose,
                                      OBS_MAT& HTVH_calib, OBS_VEC& HTVr_calib)>;
  bool UpdateObserve(ObsFunc obs);

  double GetTime() const { return current_time_; }

  SysState GetSysState() const { return SysState(current_time_, R_, p_, v_, bg_, ba_); }

  NavState GetNavState() const { return NavState(current_time_, R_, p_, v_); }

  DynamicState GetDynamicState() const { return DynamicState(current_time_, R_.R_, p_, v_, body_omega_, global_acc_); }

  KFState GetKFState() const {
    return KFState{need_converge_, GetSE3(), T_L1_IMU_, T_L2_IMU_, delta_t1_, delta_t2_};
  }

  Pose_t   GetPoseT() const { return Pose_t(current_time_, R_, p_); }

  COV GetCov() const { return P_; }

  BASIC::SE3 GetSE3() const { return BASIC::SE3(R_, p_); }

  void SetObsTime(const double obs_time) { current_obs_time_ = obs_time; }
  void SetLastObsTime(const double obs_time) { last_obs_time_ = obs_time; }

  void SetX(const SysState& x);

  void SetCov(const COV& cov){ P_ = cov; }

  void setDegeneracyOptions(const DegeneracyOptions& opts) { degen_opts_ = opts; }
  bool isDegenerate() const { return degen_detected_; }
  int  consecutiveDegenFrames() const { return consecutive_degen_frames_; }
  bool shouldResetMap() const { return reset_map_requested_; }
  void clearResetMapFlag() { reset_map_requested_ = false; }

  void setCalibEnabled(bool en) { calib_enabled_ = en; }
  bool isCalibEnabled() const { return calib_enabled_; }
  void setCalibS0(double s0) { calib_s0_ = s0; }
  BASIC::SE3 getExtrinsic1() const { return T_L1_IMU_; }
  BASIC::SE3 getExtrinsic2() const { return T_L2_IMU_; }
  double getDeltaT1() const { return delta_t1_; }
  double getDeltaT2() const { return delta_t2_; }

  BASIC::V3 GetGravity() const { return g_; }

  bool init_ = false;
  bool Predict(const IMUData& imu, DynamicState& state_imu, DynamicState& state_robot);

private:
  void BuildNoise(const Options& options);
  void Update();
  void RemapDegeneracy(const BASIC::M3& Q_trans, const BASIC::M6& HTVH, STATE& dx);
  
  bool  need_converge_  = true;
  float imu_scale_ = 1.0;
  IMUData last_imu_;
  double current_time_ = 0.0;
  double last_imu_time_ = -1.0;
  double last_obs_time_ = 0.0;
  double current_obs_time_ = 0.0;

  // nominal state
  BASIC::SO3 R_;
  BASIC::V3 p_ = BASIC::V3::Zero();
  BASIC::V3 v_ = BASIC::V3::Zero();
  BASIC::V3 bg_ = BASIC::V3::Zero();
  BASIC::V3 ba_ = BASIC::V3::Zero();
  BASIC::V3 g_{0, 0, - (BASIC::scalar)g_gravity_norm};
  BASIC::V3 global_acc_  = BASIC::V3::Zero();
  BASIC::V3 body_omega_  = BASIC::V3::Zero();

  // calibration state (nominal)
  BASIC::SE3 T_L1_IMU_;       // LiDAR1→IMU extrinsic
  BASIC::SE3 T_L2_IMU_;       // LiDAR2→IMU extrinsic
  double delta_t1_ = 0.0;     // LiDAR1 time offset
  double delta_t2_ = 0.0;     // LiDAR2 time offset

  STATE dx_ = STATE::Zero();

  COV P_ = COV::Identity();

  NOISE Q_ = NOISE::Zero();

  Options options_;

  DegeneracyOptions degen_opts_;
  bool degen_detected_ = false;
  bool calib_enabled_ = true;  // toggle for online calibration
  double calib_s0_ = 1.0;      // Jacobian normalisation factor

  // degeneracy recovery state machine
  bool was_degenerate_ = false;
  int  recovery_confirm_count_ = 0;
  int  recovery_remaining_ = 0;
  int  consecutive_degen_frames_ = 0;
  bool reset_map_requested_ = false;

  double  forward_time_ = -1;
  IMUData forward_last_imu_;
  BASIC::SO3 fw_R_;
  BASIC::V3 fw_p_ = BASIC::V3::Zero();
  BASIC::V3 fw_v_ = BASIC::V3::Zero();
};


}


#endif