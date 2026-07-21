#include "keyframe_frontend/keyframe_manager.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace keyframe_frontend
{

namespace
{

constexpr double kRadToDeg = 57.29577951308232;

double normalizeAngleRad(const double angle_rad)
{
  return std::atan2(std::sin(angle_rad), std::cos(angle_rad));
}

}  // namespace

KeyframeManager::KeyframeManager(KeyframeTriggerParams params)
: params_(std::move(params))
{
}

TriggerDecision KeyframeManager::evaluate(const nav_msgs::msg::Odometry & odom_msg) const
{
  TriggerDecision decision;

  if (keyframes_.empty()) {
    decision.create_keyframe = true;
    decision.reason_bootstrap = true;
    return decision;
  }

  const auto & last_keyframe = keyframes_.back();
  const auto current_pose = odomToIsometry(odom_msg);
  const double dt_sec = (rclcpp::Time(odom_msg.header.stamp) - last_keyframe.stamp).seconds();
  const double distance_m =
    (current_pose.translation() - last_keyframe.pose_odom_to_keyframe.translation()).norm();
  const double yaw_deg = yawDifferenceDeg(last_keyframe.pose_odom_to_keyframe, current_pose);

  decision.dt_sec = std::max(0.0, dt_sec);
  decision.distance_m = distance_m;
  decision.yaw_deg = yaw_deg;

  // 最小时间间隔是第一道防抖门，避免高频重复建帧。
  if (decision.dt_sec < params_.t_min_sec) {
    return decision;
  }

  // 若位姿变化非常小且还没到超时兜底时间，则继续等待。
  if (
    distance_m < params_.d_min_m &&
    yaw_deg < params_.yaw_min_deg &&
    decision.dt_sec < params_.t_max_sec)
  {
    return decision;
  }

  decision.reason_distance = distance_m > params_.d_thresh_m;
  decision.reason_yaw = yaw_deg > params_.yaw_thresh_deg;
  decision.reason_timeout = decision.dt_sec > params_.t_max_sec;
  decision.create_keyframe =
    decision.reason_distance || decision.reason_yaw || decision.reason_timeout;
  return decision;
}

KeyframeData KeyframeManager::createKeyframe(
  const uint8_t robot_id,
  const uint32_t keyframe_id,
  const std::string & frame_id,
  const nav_msgs::msg::Odometry & odom_msg,
  const PointCloudPtr & local_cloud) const
{
  KeyframeData keyframe;
  keyframe.robot_id = robot_id;
  keyframe.keyframe_id = keyframe_id;
  keyframe.frame_id = frame_id;
  keyframe.stamp = rclcpp::Time(odom_msg.header.stamp);
  keyframe.pose_odom_to_keyframe = odomToIsometry(odom_msg);
  keyframe.local_cloud.reset(new PointCloud(*local_cloud));
  return keyframe;
}

void KeyframeManager::addKeyframe(const KeyframeData & keyframe)
{
  keyframes_.push_back(keyframe);
}

bool KeyframeManager::hasKeyframes() const
{
  return !keyframes_.empty();
}

const KeyframeData & KeyframeManager::latest() const
{
  if (keyframes_.empty()) {
    throw std::runtime_error("No keyframe is available yet.");
  }
  return keyframes_.back();
}

const std::deque<KeyframeData> & KeyframeManager::keyframes() const
{
  return keyframes_;
}

Eigen::Isometry3d KeyframeManager::odomToIsometry(const nav_msgs::msg::Odometry & odom_msg)
{
  Eigen::Quaterniond q(
    odom_msg.pose.pose.orientation.w,
    odom_msg.pose.pose.orientation.x,
    odom_msg.pose.pose.orientation.y,
    odom_msg.pose.pose.orientation.z);
  q.normalize();

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = q.toRotationMatrix();
  transform.translation() = Eigen::Vector3d(
    odom_msg.pose.pose.position.x,
    odom_msg.pose.pose.position.y,
    odom_msg.pose.pose.position.z);
  return transform;
}

double KeyframeManager::yawDifferenceDeg(
  const Eigen::Isometry3d & lhs,
  const Eigen::Isometry3d & rhs)
{
  const double lhs_yaw = std::atan2(lhs.linear()(1, 0), lhs.linear()(0, 0));
  const double rhs_yaw = std::atan2(rhs.linear()(1, 0), rhs.linear()(0, 0));
  return std::abs(normalizeAngleRad(rhs_yaw - lhs_yaw)) * kRadToDeg;
}

}  // namespace keyframe_frontend
