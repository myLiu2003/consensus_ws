#pragma once

#include <deque>
#include <optional>
#include <string>

#include <nav_msgs/msg/odometry.hpp>

#include "keyframe_frontend/keyframe_types.hpp"

namespace keyframe_frontend
{

class KeyframeManager
{
public:
  explicit KeyframeManager(KeyframeTriggerParams params);

  TriggerDecision evaluate(const nav_msgs::msg::Odometry & odom_msg) const;

  KeyframeData createKeyframe(
    uint8_t robot_id,
    uint32_t keyframe_id,
    const std::string & frame_id,
    const nav_msgs::msg::Odometry & odom_msg,
    const PointCloudPtr & local_cloud) const;

  void addKeyframe(const KeyframeData & keyframe);

  bool hasKeyframes() const;

  const KeyframeData & latest() const;

  const std::deque<KeyframeData> & keyframes() const;

  std::optional<std::size_t> findKeyframeIndex(uint32_t keyframe_id) const;

private:
  static Eigen::Isometry3d odomToIsometry(const nav_msgs::msg::Odometry & odom_msg);
  static double yawDifferenceDeg(
    const Eigen::Isometry3d & lhs,
    const Eigen::Isometry3d & rhs);

  KeyframeTriggerParams params_;
  std::deque<KeyframeData> keyframes_;
};

}  // namespace keyframe_frontend
