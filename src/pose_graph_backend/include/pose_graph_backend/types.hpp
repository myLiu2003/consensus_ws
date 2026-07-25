#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <geometry_msgs/msg/pose.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace pose_graph_backend
{

using PointType = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<PointType>;
using PointCloudPtr = PointCloud::Ptr;

struct SubmapData
{
  uint8_t robot_id{0U};
  uint32_t keyframe_id{0U};
  geometry_msgs::msg::Pose center_local_pose;
  std::vector<uint32_t> included_keyframe_ids;
  sensor_msgs::msg::PointCloud2 cloud_msg;
  PointCloudPtr cloud{new PointCloud()};
  std::string frame_id;
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
};

struct GicpParameters
{
  double coarse_voxel_size_m{0.4};
  double fine_voxel_size_m{0.2};
  std::size_t max_points{12000U};
  double coarse_max_correspondence_distance_m{2.0};
  double fine_max_correspondence_distance_m{1.0};
  int coarse_max_iterations{30};
  int fine_max_iterations{50};
  double transformation_epsilon{1.0e-3};
  double euclidean_fitness_epsilon{1.0e-3};
  double overlap_max_distance_m{0.5};
};

struct GicpResult
{
  bool converged{false};
  bool coarse_converged{false};
  bool reverse_converged{false};
  float coarse_fitness{0.0F};
  float fitness{0.0F};
  float reverse_fitness{0.0F};
  uint32_t correspondence_count{0U};
  float overlap_ratio{0.0F};
  float forward_reverse_translation_error{0.0F};
  float forward_reverse_yaw_error_deg{0.0F};
  float translation_norm{0.0F};
  float yaw_deg{0.0F};
  Eigen::Matrix4f final_transform{Eigen::Matrix4f::Identity()};
  PointCloudPtr source_cloud{new PointCloud()};
  PointCloudPtr target_cloud{new PointCloud()};
  PointCloudPtr source_initial_cloud{new PointCloud()};
  PointCloudPtr aligned_cloud{new PointCloud()};
};

struct ConsistencyParameters
{
  std::size_t min_submap_points{1000U};
  std::size_t min_correspondences{150U};
  double min_overlap_ratio{0.30};
  double fitness_threshold_inter{0.60};
  double fitness_threshold_intra{0.35};
  double max_translation_norm_m{15.0};
  double forward_reverse_translation_thresh_m{0.5};
  double forward_reverse_yaw_thresh_deg{5.0};
  double prior_translation_error_thresh_m{2.0};
  double prior_yaw_error_deg_thresh{15.0};
  double neighbor_translation_thresh_m{0.5};
  double neighbor_yaw_thresh_deg{5.0};
  std::size_t neighbor_required_count{2U};
  uint32_t neighbor_max_keyframe_gap{6U};
};

struct ConsistencyResult
{
  bool accepted{false};
  std::string reject_reason{"not_checked"};
  float prior_translation_error{0.0F};
  float prior_yaw_error_deg{0.0F};
  float neighbor_translation_error{0.0F};
  float neighbor_yaw_error_deg{0.0F};
  uint32_t consistency_support_count{0U};
  geometry_msgs::msg::Pose relative_pose;
};

}  // namespace pose_graph_backend
