/**
 * @file parameters.hpp
 * @author WangLiansheng (lswang@mail.ecust.edu.cn)
 * @date 2023-03-14
 * @copyright Copyright (c) 2023
 */


#ifndef PARAMETERS_HPP_
#define PARAMETERS_HPP_


#include <atomic>
#include <string>
#include <vector>
#include <mutex>
#include <shared_mutex>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "basic/alias.h"
#include "basic/Manifold.h"
#include "common/log_gate.h"


namespace LI2Sup{
  
  extern const std::string g_root_dir;
  extern std::atomic<bool> g_flag_run;
  extern bool g_flg_map_init;

  /// evaluation
  extern bool g_time_eva;
  extern bool g_verbose_logging;  // enable per-frame diagnostic logs
  extern bool g_decouple_verify;  // compare 18×18 decoupled vs 32×32 full solve
  extern bool g_pose_dc_dump;     // dump per-frame H_pose + ΔC + relative extrinsic (Eq.(20) analysis)

  /// logging control (config: "日志开关 / 打印频率")
  /// 所有诊断 LOG 都由 lio.log.* 控制；WARNING 另由 g_log_warn 控制，ERROR 不受控（永远输出）。
  extern bool g_log_enable;        // 总开关: false = 关掉全部诊断日志
  extern bool g_log_warn;          // WARNING 级诊断（退化/传播守卫/丢帧/IMU 回跳）
  extern bool g_log_reloc_init;    // 重定位初始化: 载图/z 边界/多分辨率目标/收敛结果
  extern bool g_log_reloc_cand;    // 每个 yaw×z 候选的 NDT/ICP 打分
  extern bool g_log_calib;         // [Calib#] / [CalibUpdate#]
  extern bool g_log_bias;          // [Bias#] / [M20BG]
  extern bool g_log_solver;        // [SolverDiag#] / [FeatDiag#] / [Stale#] / [PU]
  extern bool g_log_input;         // [Input] 输入记账
  extern bool g_log_update_map;    // UpdateMap 进度
  extern int  g_log_calib_frames;  // [Calib#] 打印周期(帧)
  extern int  g_log_bias_frames;   // [Bias#]/[M20BG] 打印周期(帧)
  extern int  g_log_solver_frames; // 求解器诊断打印周期(帧)
  extern double g_log_input_period_sec;  // [Input] 打印周期(秒, 按雷达头时间)

  /// 统一日志闸门：item_enabled 与总开关相与；period<=0 表示该点不打印。
  inline bool log_hit(LogGate& gate, bool item_enabled, int period) {
    return gate.hit(item_enabled && g_log_enable, period);
  }

  extern bool g_save_map;
  extern bool g_if_filter;
  extern std::string g_map_name;
  extern std::string g_save_map_dir;
  extern float g_map_ds_size;
  extern int   g_pcd_save_interval;
  
  extern std::string g_imu_topic;
  extern std::string g_lidar_topic;

  extern int   g_lidar_type;       // 1: mid360, 2: hesai16, 3: velo16, 4: velo32, 5: vel_nclt, 6: ls16
  extern float g_blind2;
  extern float g_maxrange2;
  extern int   g_filter_rate;
  extern float g_intensity_thresh;   // keep all points above this intensity
  extern float g_intensity_weight;   // extra weight multiplier for high-intensity points
  extern bool  g_enable_downsample;
  extern float g_voxel_fliter_size;
  extern double g_lidar1_time_offset;
  extern double g_drop_long_frames_ms;
  extern bool   g_use_true_time_window;
  // P0-5 deskew mode: 0 = per-point IMU-interpolated deskew (the shipped path),
  // 1 = NO deskew (every point left in its own frame; a single end pose is used).
  // This exists because the geometric instrument and the estimator disagree about
  // whether the raw cloud is already compensated; the estimator is the arbiter,
  // and it needs a switch to be asked.
  extern int    g_deskew_mode;   // P0-4: start/end from true min/max   // P0-4(d): drop frames whose span exceeds this (ms); 0 = off   // clock correction for lidar1 (seconds)
  extern double g_lidar1_scan_period;   // lidar1 frame period (s); >0 enables azimuth-based deskew when the cloud has no per-point timestamps

  /// second LiDAR (dual-LiDAR fusion)
  extern bool   g_dual_lidar_enable;
  extern int    g_lidar_type2;
  extern std::string g_lidar_topic2;
  extern BASIC::SE3 g_lidar_imu2;
  extern float  g_blind2_2;
  extern float  g_maxrange2_2;
  extern int    g_filter_rate_2;
  extern double g_lidar2_weight;        // observation weight relative to lidar1 (1.0 = equal)
  extern double g_lidar2_time_offset;   // clock correction: lidar2_time + offset = imu_time (seconds)
  // true(默认)=改造前行为: 用动态补偿把 lidar2 对齐到 lidar1 的 header 域(B2 用)。
  // false=两雷达共享同一时钟时用(如 M20 同一个 rslidar_sdk): 该动态量只反映两雷达
  //       的扫描相位差(独立旋转,最大一个扫描周期),会把 lidar2 时间整体挪十几~几十 ms,
  //       此时应改为与 lidar1 同一条规则: header + g_lidar2_time_offset + g_lidar1_time_offset。
  extern bool   g_lidar2_use_dynamic_offset;
  extern double g_lidar2_scan_period;   // lidar2 frame period (s); >0 enables azimuth-based deskew

  extern int    g_imu_type;
  extern double g_gravity_norm;
  extern double g_imu_na;
  extern double g_imu_ng;
  extern double g_imu_nba;
  extern double g_imu_nbg;

  extern BASIC::SE3 g_lidar_imu;      // lidar in imu frame
  extern BASIC::SE3 g_odom_robo;      // lidar in robot frame
  extern BASIC::M3  g_lidar_robo_yaw; // lidar in robot frame rotation only yaw

  /// ROS frame ids (configurable via lio.frame.*, no longer hard-coded)
  extern std::string g_frame_world;   // global frame for odom / clouds / TF
  extern std::string g_frame_imu;     // body (imu) frame
  extern std::string g_frame_map;     // nav2 map frame
  extern std::string g_frame_odom;    // nav2 odom frame
  extern std::string g_frame_base;    // nav2 base frame

  /// hash_map
  extern std::size_t g_ivox_capacity;
  extern float       g_ivox_resolution;
  
  /// kf
  extern int g_kf_type;            // 1: ESKF, 2: InESKF.
  extern int g_kf_max_iterations;
  extern bool g_kf_align_gravity;
  extern double g_kf_quit_eps;

  /// submaps
  extern double g_submap_resolution;
  extern int    g_submap_capacity;
  
  /// output  
  extern bool g_2_robot;
  extern bool g_2_plan_env_world;
  extern bool g_2_plan_env_body;
  extern bool g_2_ml_map;
  extern bool g_visual_map;
  extern bool g_visual_dense;
  extern int  g_pub_step;
  extern bool g_lidar_qos_reliable;   // (A) 雷达订阅 QoS 与发布端 offered profile 对齐（RELIABLE）
  // (C1) 允许 lidar2-only 量测组：lidar1 断流时第二雷达不再被整段丢弃（真机鲁棒性项）
  // 默认 false = 原行为（任一解算都要求一个 lidar1 帧到位）⇒ 关时零行为变化。
  extern bool g_lidar2_only_enable;
  // (C1) **断流门控**：仅当「数据时间轴上 lidar1 已缺席 > 该秒数」才放行 lidar2-only 组。
  // 不加此门控时，10 Hz 两雷达的相位错开会让**正常段**也不断命中 else 分支 ⇒ 帧率翻倍
  // （实测 10.0 → 19.3 Hz，见 CHANGELOG 续 43）⇒ 该开关会改变标称行为，与设计意图不符。
  extern double g_lidar2_only_gap;
  extern bool g_nav2_tf;    // publish map→odom→base_link for Nav2

  /// for planner
  extern bool g_planner_enable;

  /// Define the hybrid residual formulation.
  enum ResidualType{
    PROB = 1,     // Probabilistic residual
    P2P  = 2,     // Point-to-plane residual
    MIX  = 3      // Hybrid residual (probabilistic + point-to-plane)
  };
  extern ResidualType g_residual_type;

  /// online calibration
  extern bool   g_enable_online_calib;
  extern double g_calib_ext_rot_rw;    // extrinsic rotation random walk (rad/√s)
  extern double g_calib_ext_trans_rw;  // extrinsic translation random walk (m/√s)
  extern double g_calib_dt_rw;         // time offset random walk (s/√s)

  /// edge (point-to-line) feature
  extern bool   g_enable_edge_feature;
  extern double g_edge_weight;
  extern double g_edge_line_dist_thresh;   // max distance to fitted line (neighbors & scan point)
  extern double g_edge_linearity_ratio;    // λ_max / λ_2nd > this in fit_line_from_points
  extern double g_high_intensity_edge_boost;  // extra multiplier for high-intensity edge features

  /// P lower bound (covariance clamp)
  extern bool   g_p_lower_bound_enabled;  // enable P lower bound clamping
  extern double g_p_lower_bound_trans;    // min σ for position (m), e.g. 0.05 = 5cm

  /// degeneracy handling
  extern bool   g_degeneracy_enabled;
  extern double g_degeneracy_max_stddev;       // posterior σ > this (meters) → degenerate
  extern double g_degeneracy_htvh_cond_thresh; // HTVH trans κ > this → constraint imbalance

  /// structural regularity prior — two independent channels
  extern bool   g_structure_wall_enabled;       // vertical planes → constrain roll
  extern bool   g_structure_ground_enabled;     // floor longitudinal component → constrain pitch
  extern bool   g_structure_ground_grade_auto;  // capture the terrain grade in the init stationary segment
  extern double g_structure_ground_grade_deg;   // manual reference longitudinal grade (deg), used when auto is off
  extern double g_structure_ground_weight;      // information weight of the scan-based grade residual
  extern double g_structure_max_angle_deg;      // classification: max deviation from an axis (deg)

  /// for relocation
  extern bool g_update_map;
  extern double g_init_px, g_init_py, g_init_pz, g_init_roll, g_init_pitch, g_init_yaw;

  /// relocation registration parameters
  extern double g_reloc_target_ds_size;     // downsample leaf size for target map (0=skip)
  extern double g_reloc_ndt_resolution;     // fine NDT resolution (final stage)
  extern double g_reloc_icp_max_corr;       // ICP max correspondence distance
  extern int    g_reloc_icp_max_iter;       // ICP max iterations
  extern double g_reloc_fitness_thresh;     // ICP fitness score threshold
  extern bool   g_reloc_multi_yaw_search;   // try multiple yaw angles on failure
  extern double g_reloc_yaw_step_deg;       // yaw step for multi-yaw search
  extern int    g_reloc_yaw_search_count;   // number of yaw candidates per side

  /// multi-resolution NDT (coarse-to-fine)
  extern double g_reloc_coarse_ds_size;         // coarse stage: target downsample leaf
  extern double g_reloc_coarse_ndt_resolution;  // coarse stage: NDT voxel resolution
  extern double g_reloc_medium_ds_size;         // medium stage: target downsample leaf
  extern double g_reloc_medium_ndt_resolution;  // medium stage: NDT voxel resolution
  extern bool   g_reloc_enable_multi_res;       // enable multi-resolution cascade

  /// relocation robustness
  extern int    g_reloc_init_frames;         // number of lidar frames to accumulate for relocation
  extern double g_reloc_z_search_range;      // ±z range for fallback search (meters)
  extern double g_reloc_z_search_step;       // z step for fallback search (meters)
  extern double g_reloc_min_overlap_ratio;   // minimum ICP inlier ratio (0-1), 0=disabled

}

#endif
