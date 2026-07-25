#pragma once

#include <map>
#include <utility>

#include <Eigen/Geometry>
#include <map_consensus_msgs/msg/loop_candidate.hpp>

#include "pose_graph_backend/types.hpp"

namespace pose_graph_backend
{

class ConsistencyChecker
{
public:
  explicit ConsistencyChecker(const ConsistencyParameters & params);

  ConsistencyResult evaluate(
    const map_consensus_msgs::msg::LoopCandidate & candidate_msg,
    const GicpResult & gicp_result);

private:
  struct NeighborHistory
  {
    Eigen::Isometry3d canonical_transform{Eigen::Isometry3d::Identity()};
    uint32_t low_robot_keyframe_id{0U};
    uint32_t high_robot_keyframe_id{0U};
    std::size_t support_count{0U};
  };

  static Eigen::Isometry3d poseToIsometry(const geometry_msgs::msg::Pose & pose_msg);
  static geometry_msgs::msg::Pose isometryToPose(const Eigen::Isometry3d & transform);
  static Eigen::Isometry3d canonicalInterRobotTransform(
    const map_consensus_msgs::msg::LoopCandidate & candidate_msg,
    const Eigen::Isometry3d & estimated_transform);
  static double yawFromIsometryDeg(const Eigen::Isometry3d & transform);
  static double normalizeAngleDeg(double angle_deg);
  static uint32_t keyframeGap(uint32_t lhs, uint32_t rhs);

  ConsistencyParameters params_;
  std::map<std::pair<uint8_t, uint8_t>, NeighborHistory> neighbor_history_;
};

}  // namespace pose_graph_backend
