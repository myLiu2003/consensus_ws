#include "pose_graph_backend/consistency_checker.hpp"

#include <algorithm>
#include <cmath>

#include <Eigen/Geometry>

namespace pose_graph_backend
{

ConsistencyChecker::ConsistencyChecker(const ConsistencyParameters & params)
: params_(params)
{
}

ConsistencyResult ConsistencyChecker::evaluate(
  const map_consensus_msgs::msg::LoopCandidate & candidate_msg,
  const GicpResult & gicp_result)
{
  ConsistencyResult result;
  Eigen::Isometry3d estimated_transform = Eigen::Isometry3d::Identity();
  estimated_transform.matrix() = gicp_result.final_transform.cast<double>();
  result.relative_pose = isometryToPose(estimated_transform);

  if (gicp_result.source_cloud->size() < params_.min_submap_points ||
    gicp_result.target_cloud->size() < params_.min_submap_points)
  {
    result.reject_reason = "submap_too_sparse";
    return result;
  }
  if (!gicp_result.coarse_converged) {
    result.reject_reason = "gicp_coarse_not_converged";
    return result;
  }
  if (!gicp_result.converged) {
    result.reject_reason = "gicp_fine_not_converged";
    return result;
  }
  if (!gicp_result.reverse_converged) {
    result.reject_reason = "gicp_reverse_not_converged";
    return result;
  }
  if (gicp_result.correspondence_count < params_.min_correspondences) {
    result.reject_reason = "too_few_correspondences";
    return result;
  }
  if (gicp_result.overlap_ratio < params_.min_overlap_ratio) {
    result.reject_reason = "overlap_too_small";
    return result;
  }
  if (gicp_result.translation_norm > params_.max_translation_norm_m) {
    result.reject_reason = "gicp_translation_too_large";
    return result;
  }
  if (gicp_result.forward_reverse_translation_error >
    params_.forward_reverse_translation_thresh_m)
  {
    result.reject_reason = "forward_reverse_translation_mismatch";
    return result;
  }
  if (gicp_result.forward_reverse_yaw_error_deg > params_.forward_reverse_yaw_thresh_deg) {
    result.reject_reason = "forward_reverse_yaw_mismatch";
    return result;
  }

  const bool intra_robot = candidate_msg.query_robot_id == candidate_msg.candidate_robot_id;
  const double fitness_threshold =
    intra_robot ? params_.fitness_threshold_intra : params_.fitness_threshold_inter;
  if (!std::isfinite(gicp_result.fitness) || gicp_result.fitness > fitness_threshold) {
    result.reject_reason = "gicp_fitness_too_large";
    return result;
  }

  if (intra_robot) {
    const Eigen::Isometry3d query_pose = poseToIsometry(candidate_msg.query_local_pose);
    const Eigen::Isometry3d candidate_pose = poseToIsometry(candidate_msg.candidate_local_pose);
    const Eigen::Isometry3d prior_transform = candidate_pose.inverse() * query_pose;
    result.prior_translation_error = static_cast<float>(
      (estimated_transform.translation() - prior_transform.translation()).norm());
    result.prior_yaw_error_deg = static_cast<float>(std::abs(normalizeAngleDeg(
          yawFromIsometryDeg(estimated_transform) - yawFromIsometryDeg(prior_transform))));

    if (result.prior_translation_error > params_.prior_translation_error_thresh_m) {
      result.reject_reason = "prior_translation_mismatch";
      return result;
    }
    if (result.prior_yaw_error_deg > params_.prior_yaw_error_deg_thresh) {
      result.reject_reason = "prior_yaw_mismatch";
      return result;
    }

    result.consistency_support_count = 1U;
    result.accepted = true;
    result.reject_reason = "accepted";
    return result;
  }

  result.prior_translation_error = -1.0F;
  result.prior_yaw_error_deg = -1.0F;

  const uint8_t low_robot = std::min(candidate_msg.query_robot_id, candidate_msg.candidate_robot_id);
  const uint8_t high_robot = std::max(candidate_msg.query_robot_id, candidate_msg.candidate_robot_id);
  const bool query_is_low = candidate_msg.query_robot_id == low_robot;
  const uint32_t low_keyframe_id = query_is_low ?
    candidate_msg.query_keyframe_id : candidate_msg.candidate_keyframe_id;
  const uint32_t high_keyframe_id = query_is_low ?
    candidate_msg.candidate_keyframe_id : candidate_msg.query_keyframe_id;
  const auto canonical_transform = canonicalInterRobotTransform(candidate_msg, estimated_transform);
  auto & history = neighbor_history_[{low_robot, high_robot}];

  const bool has_history = history.support_count > 0U;
  const bool adjacent = has_history &&
    keyframeGap(low_keyframe_id, history.low_robot_keyframe_id) <=
    params_.neighbor_max_keyframe_gap &&
    keyframeGap(high_keyframe_id, history.high_robot_keyframe_id) <=
    params_.neighbor_max_keyframe_gap &&
    (low_keyframe_id != history.low_robot_keyframe_id ||
    high_keyframe_id != history.high_robot_keyframe_id);

  bool transform_consistent = false;
  if (adjacent) {
    const Eigen::Isometry3d delta = history.canonical_transform.inverse() * canonical_transform;
    result.neighbor_translation_error = static_cast<float>(delta.translation().norm());
    result.neighbor_yaw_error_deg = static_cast<float>(
      std::abs(normalizeAngleDeg(yawFromIsometryDeg(delta))));
    transform_consistent =
      result.neighbor_translation_error <= params_.neighbor_translation_thresh_m &&
      result.neighbor_yaw_error_deg <= params_.neighbor_yaw_thresh_deg;
  }

  if (adjacent && transform_consistent) {
    history.support_count = std::min(
      history.support_count + 1U, params_.neighbor_required_count);
  } else {
    history.support_count = 1U;
  }
  history.canonical_transform = canonical_transform;
  history.low_robot_keyframe_id = low_keyframe_id;
  history.high_robot_keyframe_id = high_keyframe_id;
  result.consistency_support_count = static_cast<uint32_t>(history.support_count);

  if (history.support_count < params_.neighbor_required_count) {
    result.reject_reason = adjacent && !transform_consistent ?
      "neighbor_transform_inconsistent" : "awaiting_neighbor_consistency";
    return result;
  }

  result.accepted = true;
  result.reject_reason = "accepted";
  return result;
}

Eigen::Isometry3d ConsistencyChecker::poseToIsometry(const geometry_msgs::msg::Pose & pose_msg)
{
  Eigen::Quaterniond q(
    pose_msg.orientation.w, pose_msg.orientation.x,
    pose_msg.orientation.y, pose_msg.orientation.z);
  if (q.norm() < 1.0e-9) {
    q = Eigen::Quaterniond::Identity();
  } else {
    q.normalize();
  }

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = q.toRotationMatrix();
  transform.translation() = Eigen::Vector3d(
    pose_msg.position.x, pose_msg.position.y, pose_msg.position.z);
  return transform;
}

geometry_msgs::msg::Pose ConsistencyChecker::isometryToPose(
  const Eigen::Isometry3d & transform)
{
  geometry_msgs::msg::Pose pose_msg;
  pose_msg.position.x = transform.translation().x();
  pose_msg.position.y = transform.translation().y();
  pose_msg.position.z = transform.translation().z();
  const Eigen::Quaterniond q(transform.rotation());
  pose_msg.orientation.x = q.x();
  pose_msg.orientation.y = q.y();
  pose_msg.orientation.z = q.z();
  pose_msg.orientation.w = q.w();
  return pose_msg;
}

Eigen::Isometry3d ConsistencyChecker::canonicalInterRobotTransform(
  const map_consensus_msgs::msg::LoopCandidate & candidate_msg,
  const Eigen::Isometry3d & estimated_transform)
{
  const Eigen::Isometry3d query_pose = poseToIsometry(candidate_msg.query_local_pose);
  const Eigen::Isometry3d candidate_pose = poseToIsometry(candidate_msg.candidate_local_pose);
  const Eigen::Isometry3d candidate_odom_from_query_odom =
    candidate_pose * estimated_transform * query_pose.inverse();
  if (candidate_msg.query_robot_id < candidate_msg.candidate_robot_id) {
    return candidate_odom_from_query_odom;
  }
  return candidate_odom_from_query_odom.inverse();
}

double ConsistencyChecker::yawFromIsometryDeg(const Eigen::Isometry3d & transform)
{
  return std::atan2(transform.rotation()(1, 0), transform.rotation()(0, 0)) *
         180.0 / 3.14159265358979323846;
}

double ConsistencyChecker::normalizeAngleDeg(double angle_deg)
{
  while (angle_deg > 180.0) {
    angle_deg -= 360.0;
  }
  while (angle_deg < -180.0) {
    angle_deg += 360.0;
  }
  return angle_deg;
}

uint32_t ConsistencyChecker::keyframeGap(uint32_t lhs, uint32_t rhs)
{
  return lhs > rhs ? lhs - rhs : rhs - lhs;
}

}  // namespace pose_graph_backend
