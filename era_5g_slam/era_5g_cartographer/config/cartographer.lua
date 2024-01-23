include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "robot/imu_link",
  published_frame = "robot/odom",
  odom_frame = "robot/odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1

MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.num_background_threads = 7
POSE_GRAPH.optimization_problem.huber_scale = 5e2
POSE_GRAPH.optimize_every_n_nodes = 60
POSE_GRAPH.constraint_builder.sampling_ratio = 0.03
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 10
POSE_GRAPH.constraint_builder.min_score = 0.6
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.65

-- Only sampling_ratio * total_nodes_in_search_window will be considered to create matches
-- Decrease to reduce CPU usage, increase to increment potential matches between nodes
POSE_GRAPH.constraint_builder.sampling_ratio = 0.1

-- Sets the weight of odometry in pose estimation
-- Reduce to increase scan_matcher belief (position estimation will jitter with very small values)
-- Reduce in enviroments where odometry can't be trusted
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.translation_weight = 500
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 700

-- Setting them to true is creating problems even in 2D environments. Keep them deactivated
POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.only_optimize_yaw = false
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.only_optimize_yaw = false
POSE_GRAPH.optimization_problem.use_online_imu_extrinsics_in_3d = false
POSE_GRAPH.optimization_problem.fix_z_in_3d = false

return options