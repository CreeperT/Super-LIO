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


namespace LI2Sup{
  
  extern const std::string g_root_dir;
  extern std::atomic<bool> g_flag_run;
  extern bool g_flg_map_init;

  /// evaluation
  extern bool g_time_eva;

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
  extern double g_lidar1_time_offset;   // clock correction for lidar1 (seconds)

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

  extern int    g_imu_type;
  extern double g_gravity_norm;
  extern double g_imu_na;
  extern double g_imu_ng;
  extern double g_imu_nba;
  extern double g_imu_nbg;

  extern BASIC::SE3 g_lidar_imu;      // lidar in imu frame
  extern BASIC::SE3 g_odom_robo;      // lidar in robot frame
  extern BASIC::M3  g_lidar_robo_yaw; // lidar in robot frame rotation only yaw

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
