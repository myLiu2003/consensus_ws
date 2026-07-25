#pragma once

#include <array>
#include <memory>
#include <string>

#include <map_consensus_msgs/srv/get_submap.hpp>
#include <rclcpp/rclcpp.hpp>

#include "pose_graph_backend/types.hpp"

namespace pose_graph_backend
{

class SubmapClient
{
public:
  SubmapClient(
    rclcpp::Node * node,
    const std::array<std::string, 4> & service_names,
    rclcpp::CallbackGroup::SharedPtr response_callback_group);

  bool fetchSubmap(
    uint8_t robot_id,
    uint32_t keyframe_id,
    double timeout_sec,
    SubmapData & output,
    std::string & error_message);

private:
  rclcpp::Node * node_;
  std::array<std::string, 4> service_names_;
  rclcpp::CallbackGroup::SharedPtr response_callback_group_;
  std::array<rclcpp::Client<map_consensus_msgs::srv::GetSubmap>::SharedPtr, 4> clients_;
};

}  // namespace pose_graph_backend
