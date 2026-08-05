

#include "lio/params.h"

using namespace std;
using namespace BASIC;

namespace LI2Sup{

  const std::string g_root_dir = std::string(ROOT);
  std::atomic<bool> g_flag_run = true; 
  bool g_flg_map_init = true;

  /// evaluation
  bool g_time_eva = false;
  bool g_verbose_logging = false;  // per-frame diagnostic logs (off by default)
  bool g_decouple_verify = false;  // compare 18×18 decoupled vs 32×32 full solve

  bool   g_save_map;
  bool   g_if_filter; 
  string g_save_map_dir;
  string g_map_name;
  float  g_map_ds_size;
  int    g_pcd_save_interval;
  
  string g_imu_topic;
  string g_lidar_topic;

  int    g_lidar_type;
  float  g_blind2;
  float  g_maxrange2;
  int    g_filter_rate;
  float  g_intensity_thresh = 200.0f;   // keep all points above this (max ~255)
  float  g_intensity_weight = 2.0f;     // extra weight factor for high-intensity
  bool   g_enable_downsample;
  float  g_voxel_fliter_size;
  double g_lidar1_time_offset = 0.0;

  /// second LiDAR (dual-LiDAR fusion)
  bool   g_dual_lidar_enable = false;
  int    g_lidar_type2 = 1;         // default: Livox Mid-360
  std::string g_lidar_topic2 = "/livox/lidar";
  SE3    g_lidar_imu2;
  float  g_blind2_2 = 0.09f;        // 0.3^2
  float  g_maxrange2_2 = 10000.0f;  // 100^2
  int    g_filter_rate_2 = 3;
  double g_lidar2_weight = 1.0;
  double g_lidar2_time_offset = 0.0;

  int    g_imu_type;
  double g_gravity_norm = 9.7946;
  double g_imu_na;
  double g_imu_ng;
  double g_imu_nba;
  double g_imu_nbg;

  SE3 g_lidar_imu;
  SE3 g_odom_robo;
  M3  g_lidar_robo_yaw;

  /// hash_map
  std::size_t g_ivox_capacity = 100000;
  float       g_ivox_resolution = 0.5;

  /// kf
  int g_kf_type = 1;                // 1: ESKF, 2: InESKF
  int g_kf_max_iterations = 4;
  bool g_kf_align_gravity = true;
  double g_kf_quit_eps;

  /// submap 
  double g_submap_resolution;
  int    g_submap_capacity;

  /// output
  bool g_2_robot    = false;
  bool g_2_plan_env_world = false; 
  bool g_2_plan_env_body  = false;
  bool g_2_ml_map = false;
  bool g_visual_map = true;
  bool g_visual_dense = false;
  int  g_pub_step;
  bool g_nav2_tf    = false;

  /// for planner
  bool g_planner_enable;

  ResidualType g_residual_type = PROB;

  /// P lower bound (covariance clamp)
  bool   g_p_lower_bound_enabled = false;
  double g_p_lower_bound_trans = 0.05;  // σ_pos ≥ 5cm

  /// online calibration
  bool   g_enable_online_calib = true;
  double g_calib_ext_rot_rw   = 1.7e-4;   // ~0.01°/√s
  double g_calib_ext_trans_rw = 1.0e-4;   // ~0.1mm/√s
  double g_calib_dt_rw        = 1.0e-6;   // ~1μs/√s

  /// edge (point-to-line) feature
  bool   g_enable_edge_feature = true;
  double g_edge_weight = 100.0;
  double g_edge_line_dist_thresh = 0.05;
  double g_edge_linearity_ratio = 5.0;
  double g_high_intensity_edge_boost = 3.0;

  /// degeneracy handling
  bool   g_degeneracy_enabled = true;
  double g_degeneracy_max_stddev = 0.5;        // meters
  double g_degeneracy_htvh_cond_thresh = 100.0;

  /// for relocation
  bool g_update_map = false;
  double g_init_px, g_init_py, g_init_pz, g_init_roll, g_init_pitch, g_init_yaw;

  /// relocation registration parameters
  double g_reloc_target_ds_size = 0.5;
  double g_reloc_ndt_resolution = 1.0;
  double g_reloc_icp_max_corr = 8.0;
  int    g_reloc_icp_max_iter = 40;
  double g_reloc_fitness_thresh = 1.5;
  bool   g_reloc_multi_yaw_search = false;
  double g_reloc_yaw_step_deg = 30.0;
  int    g_reloc_yaw_search_count = 6;

  /// multi-resolution NDT (coarse-to-fine)
  double g_reloc_coarse_ds_size = 1.5;
  double g_reloc_coarse_ndt_resolution = 5.0;
  double g_reloc_medium_ds_size = 0.8;
  double g_reloc_medium_ndt_resolution = 2.0;
  bool   g_reloc_enable_multi_res = true;

  /// relocation robustness
  int    g_reloc_init_frames = 5;
  double g_reloc_z_search_range = 2.0;
  double g_reloc_z_search_step = 1.0;
  double g_reloc_min_overlap_ratio = 0.0;

}