#pragma once

#include <deque>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>

namespace keyframe_frontend
{

using PointType = pcl::PointXYZI;
using PointCloud = pcl::PointCloud<PointType>;
using PointCloudPtr = PointCloud::Ptr;
using PointCloudConstPtr = PointCloud::ConstPtr;

struct TriggerDecision
{
  bool create_keyframe{false};
  bool reason_bootstrap{false};
  bool reason_distance{false};
  bool reason_yaw{false};
  bool reason_timeout{false};
  double dt_sec{0.0};
  double distance_m{0.0};
  double yaw_deg{0.0};
};

struct KeyframeData
{
  uint32_t keyframe_id{0U};
  uint8_t robot_id{0U};
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  std::string frame_id;
  Eigen::Isometry3d pose_odom_to_keyframe{Eigen::Isometry3d::Identity()};
  PointCloudPtr local_cloud{new PointCloud()};
};

struct KeyframeTriggerParams
{
  double d_thresh_m{0.6};
  double yaw_thresh_deg{10.0};
  double t_max_sec{1.2};

  double t_min_sec{0.3};
  double d_min_m{0.15};
  double yaw_min_deg{3.0};
};

struct SubmapParams
{
  int window_size{3};
  double range_min_m{0.8};
  double range_max_m{40.0};
  double body_exclusion_radius_m{0.8};
  double voxel_size_m{0.20};
  std::size_t max_points{25000U};
};

}  // namespace keyframe_frontend
