#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/time.hpp>

namespace loop_consensus
{

struct DescriptorEntry
{
  uint8_t robot_id{0U};
  uint32_t keyframe_id{0U};
  uint32_t graph_version{0U};
  std::size_t robot_sequence{0U};
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  std::string frame_id;
  geometry_msgs::msg::Pose local_pose;
  Eigen::MatrixXf scan_context;
  Eigen::VectorXf ring_key;
  Eigen::RowVectorXf sector_key;
};

struct CandidateResult
{
  DescriptorEntry query;
  DescriptorEntry candidate;
  std::size_t rank{0U};
  float distance{0.0F};
  int best_shift{0};
  float yaw_initial_rad{0.0F};
  bool intra_robot{false};
};

}  // namespace loop_consensus
