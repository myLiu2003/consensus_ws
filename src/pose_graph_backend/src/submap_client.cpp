#include "pose_graph_backend/submap_client.hpp"

#include <chrono>
#include <utility>

#include <pcl_conversions/pcl_conversions.h>

namespace pose_graph_backend
{

using namespace std::chrono_literals;

SubmapClient::SubmapClient(
  rclcpp::Node * node,
  const std::array<std::string, 4> & service_names,
  rclcpp::CallbackGroup::SharedPtr response_callback_group)
: node_(node),
  service_names_(service_names),
  response_callback_group_(std::move(response_callback_group))
{
  for (std::size_t robot_id = 1; robot_id < service_names_.size(); ++robot_id) {
    clients_[robot_id] = node_->create_client<map_consensus_msgs::srv::GetSubmap>(
      service_names_[robot_id],
      rmw_qos_profile_services_default,
      response_callback_group_);
  }
}

bool SubmapClient::fetchSubmap(
  uint8_t robot_id,
  uint32_t keyframe_id,
  double timeout_sec,
  SubmapData & output,
  std::string & error_message)
{
  if (robot_id == 0U || robot_id >= clients_.size()) {
    error_message = "invalid robot_id=" + std::to_string(robot_id);
    return false;
  }

  auto client = clients_[robot_id];
  if (!client) {
    error_message = "submap client not initialized for robot_id=" + std::to_string(robot_id);
    return false;
  }

  const auto timeout = std::chrono::duration<double>(timeout_sec);
  if (!client->wait_for_service(timeout)) {
    error_message = "submap service unavailable: " + service_names_[robot_id];
    return false;
  }

  auto request = std::make_shared<map_consensus_msgs::srv::GetSubmap::Request>();
  request->keyframe_id = keyframe_id;
  auto future = client->async_send_request(request);
  if (future.wait_for(timeout) != std::future_status::ready) {
    error_message = "submap request timed out for robot_id=" + std::to_string(robot_id) +
      " keyframe_id=" + std::to_string(keyframe_id);
    return false;
  }

  const auto response = future.get();
  if (!response->success) {
    error_message = response->message;
    return false;
  }

  output.robot_id = robot_id;
  output.keyframe_id = keyframe_id;
  output.center_local_pose = response->center_local_pose;
  output.included_keyframe_ids = response->included_keyframe_ids;
  output.cloud_msg = response->submap;
  output.frame_id = response->submap.header.frame_id;
  output.stamp = rclcpp::Time(response->submap.header.stamp);
  output.cloud.reset(new PointCloud());
  pcl::fromROSMsg(response->submap, *output.cloud);

  error_message.clear();
  return true;
}

}  // namespace pose_graph_backend
