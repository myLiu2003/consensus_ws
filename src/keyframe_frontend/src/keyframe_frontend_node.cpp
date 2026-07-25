#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <map_consensus_msgs/msg/keyframe.hpp>
#include <map_consensus_msgs/srv/get_submap.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "keyframe_frontend/keyframe_manager.hpp"
#include "keyframe_frontend/submap_builder.hpp"

namespace keyframe_frontend
{

class KeyframeFrontendNode : public rclcpp::Node
{
public:
  KeyframeFrontendNode()
  : Node("keyframe_frontend_node"),
    trigger_params_(loadTriggerParams()),
    submap_params_(loadSubmapParams()),
    keyframe_manager_(trigger_params_),
    submap_builder_(submap_params_)
  {
    robot_id_ = static_cast<uint8_t>(declare_parameter<int>("robot_id", 1));
    sync_tolerance_sec_ = declare_parameter<double>("sync_tolerance_sec", 0.12);
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/uav1/fast_lio/odometry");
    cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/uav1/fast_lio/cloud_registered");
    keyframe_topic_ = declare_parameter<std::string>(
      "keyframe_topic", "/uav1/consensus/keyframe");
    submap_topic_ = declare_parameter<std::string>(
      "submap_topic", "/uav1/consensus/submap_cloud");
    submap_service_name_ = declare_parameter<std::string>(
      "submap_service_name", "/uav1/consensus/get_submap");
    keyframe_path_topic_ = declare_parameter<std::string>(
      "keyframe_path_topic", "/uav1/consensus/keyframe_path");
    keyframe_marker_topic_ = declare_parameter<std::string>(
      "keyframe_marker_topic", "/uav1/consensus/keyframe_markers");
    publish_submap_on_request_ = declare_parameter<bool>("publish_submap_on_request", true);
    keyframe_marker_scale_m_ = declare_parameter<double>("keyframe_marker_scale", 0.35);

    const auto qos = rclcpp::SensorDataQoS();
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, qos,
      std::bind(&KeyframeFrontendNode::odomCallback, this, std::placeholders::_1));
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, qos,
      std::bind(&KeyframeFrontendNode::cloudCallback, this, std::placeholders::_1));

    keyframe_pub_ = create_publisher<map_consensus_msgs::msg::Keyframe>(keyframe_topic_, 10);
    submap_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(submap_topic_, 10);
    keyframe_path_pub_ = create_publisher<nav_msgs::msg::Path>(keyframe_path_topic_, 10);
    keyframe_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      keyframe_marker_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
    submap_service_ = create_service<map_consensus_msgs::srv::GetSubmap>(
      submap_service_name_,
      std::bind(
        &KeyframeFrontendNode::handleGetSubmapRequest,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    keyframe_path_.header.frame_id = "odom_uav" + std::to_string(robot_id_);

    RCLCPP_INFO(
      get_logger(),
      "keyframe_frontend started, robot_id=%u, odom=%s, cloud=%s, markers=%s, get_submap=%s",
      robot_id_,
      odom_topic_.c_str(),
      cloud_topic_.c_str(),
      keyframe_marker_topic_.c_str(),
      submap_service_name_.c_str());
  }

private:
  KeyframeTriggerParams loadTriggerParams()
  {
    KeyframeTriggerParams params;
    params.d_thresh_m = declare_parameter<double>("d_thresh", 0.6);
    params.yaw_thresh_deg = declare_parameter<double>("yaw_thresh_deg", 10.0);
    params.t_max_sec = declare_parameter<double>("t_max", 1.2);
    params.t_min_sec = declare_parameter<double>("t_min", 0.3);
    params.d_min_m = declare_parameter<double>("d_min", 0.15);
    params.yaw_min_deg = declare_parameter<double>("yaw_min_deg", 3.0);
    return params;
  }

  SubmapParams loadSubmapParams()
  {
    SubmapParams params;
    params.window_size = declare_parameter<int>("submap_window", 3);
    params.range_min_m = declare_parameter<double>("range_min", 0.8);
    params.range_max_m = declare_parameter<double>("range_max", 40.0);
    params.body_exclusion_radius_m = declare_parameter<double>("body_exclusion_radius", 0.8);
    params.voxel_size_m = declare_parameter<double>("voxel_size", 0.20);
    params.max_points = static_cast<std::size_t>(declare_parameter<int>("max_points", 25000));
    return params;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::scoped_lock<std::mutex> lock(latest_odom_mutex_);
    latest_odom_ = msg;
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    nav_msgs::msg::Odometry::SharedPtr odom_msg;
    {
      std::scoped_lock<std::mutex> lock(latest_odom_mutex_);
      odom_msg = latest_odom_;
    }

    if (!odom_msg) {
      return;
    }

    const double time_diff = std::abs(
      (rclcpp::Time(msg->header.stamp) - rclcpp::Time(odom_msg->header.stamp)).seconds());
    if (time_diff > sync_tolerance_sec_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Skip cloud because odom-cloud time diff %.3f s exceeds tolerance %.3f s.",
        time_diff, sync_tolerance_sec_);
      return;
    }

    const TriggerDecision decision = keyframe_manager_.evaluate(*odom_msg);
    if (!decision.create_keyframe) {
      return;
    }

    PointCloudPtr cloud_in_odom(new PointCloud());
    pcl::fromROSMsg(*msg, *cloud_in_odom);

    const Eigen::Isometry3d pose_odom_to_body = odomToIsometry(*odom_msg);
    PointCloudPtr cloud_in_keyframe(new PointCloud());
    pcl::transformPointCloud(
      *cloud_in_odom,
      *cloud_in_keyframe,
      pose_odom_to_body.inverse().matrix().cast<float>());

    auto local_cloud = submap_builder_.preprocessSingleFrame(cloud_in_keyframe);
    if (local_cloud->empty()) {
      RCLCPP_WARN(
        get_logger(),
        "Candidate keyframe rejected because the local cloud is empty after preprocessing.");
      return;
    }

    const uint32_t keyframe_id = next_keyframe_id_++;
    const std::string frame_id =
      "uav" + std::to_string(robot_id_) + "_keyframe_" + std::to_string(keyframe_id);
    auto keyframe =
      keyframe_manager_.createKeyframe(robot_id_, keyframe_id, frame_id, *odom_msg, local_cloud);
    keyframe_manager_.addKeyframe(keyframe);

    publishKeyframe(keyframe);
    publishKeyframePath(*odom_msg);
    publishKeyframeMarkers(keyframe);

    RCLCPP_INFO(
      get_logger(),
      "Created keyframe %u | dt=%.2f s, dp=%.2f m, dyaw=%.2f deg | reason=%s",
      keyframe_id, decision.dt_sec, decision.distance_m, decision.yaw_deg,
      decisionSummary(decision).c_str());
  }

  void publishKeyframe(const KeyframeData & keyframe)
  {
    map_consensus_msgs::msg::Keyframe msg;
    msg.header.stamp = keyframe.stamp;
    msg.header.frame_id = keyframe.frame_id;
    msg.robot_id = keyframe.robot_id;
    msg.keyframe_id = keyframe.keyframe_id;
    msg.graph_version = 0U;

    msg.local_pose.position.x = keyframe.pose_odom_to_keyframe.translation().x();
    msg.local_pose.position.y = keyframe.pose_odom_to_keyframe.translation().y();
    msg.local_pose.position.z = keyframe.pose_odom_to_keyframe.translation().z();

    const Eigen::Quaterniond q(keyframe.pose_odom_to_keyframe.rotation());
    msg.local_pose.orientation.x = q.x();
    msg.local_pose.orientation.y = q.y();
    msg.local_pose.orientation.z = q.z();
    msg.local_pose.orientation.w = q.w();

    pcl::toROSMsg(*keyframe.local_cloud, msg.cloud);
    msg.cloud.header.stamp = keyframe.stamp;
    msg.cloud.header.frame_id = keyframe.frame_id;

    keyframe_pub_->publish(msg);
  }

  void handleGetSubmapRequest(
    const std::shared_ptr<map_consensus_msgs::srv::GetSubmap::Request> request,
    std::shared_ptr<map_consensus_msgs::srv::GetSubmap::Response> response)
  {
    const auto center_index = keyframe_manager_.findKeyframeIndex(request->keyframe_id);
    if (!center_index.has_value()) {
      response->success = false;
      response->message =
        "keyframe_id=" + std::to_string(request->keyframe_id) + " is not available";
      RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
      return;
    }

    const auto submap_result = buildSubmapResponse(center_index.value());
    response->success = true;
    response->message = submap_result.message;
    response->center_keyframe_id = submap_result.center_keyframe_id;
    response->included_keyframe_ids = submap_result.included_keyframe_ids;
    response->center_local_pose = submap_result.center_local_pose;
    response->submap = submap_result.submap;

    if (publish_submap_on_request_) {
      submap_pub_->publish(submap_result.submap);
    }

    RCLCPP_INFO(
      get_logger(),
      "Built on-demand submap for keyframe %u using %zu frames.",
      response->center_keyframe_id,
      response->included_keyframe_ids.size());
  }

  void publishKeyframePath(const nav_msgs::msg::Odometry & odom_msg)
  {
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header = odom_msg.header;
    pose_stamped.pose = odom_msg.pose.pose;
    keyframe_path_.header.stamp = odom_msg.header.stamp;
    keyframe_path_.header.frame_id = odom_msg.header.frame_id;
    keyframe_path_.poses.push_back(pose_stamped);
    keyframe_path_pub_->publish(keyframe_path_);
  }

  void publishKeyframeMarkers(const KeyframeData & keyframe)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = keyframe.stamp;
    marker.header.frame_id = keyframe_path_.header.frame_id;
    marker.ns = "keyframes";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = keyframe_marker_scale_m_;
    marker.scale.y = keyframe_marker_scale_m_;
    marker.scale.z = keyframe_marker_scale_m_;
    marker.color.r = 1.0F;
    marker.color.g = 0.15F;
    marker.color.b = 0.15F;
    marker.color.a = 1.0F;
    marker.lifetime = rclcpp::Duration(0, 0);
    marker.frame_locked = false;

    for (const auto & stored_keyframe : keyframe_manager_.keyframes()) {
      geometry_msgs::msg::Point point;
      point.x = stored_keyframe.pose_odom_to_keyframe.translation().x();
      point.y = stored_keyframe.pose_odom_to_keyframe.translation().y();
      point.z = stored_keyframe.pose_odom_to_keyframe.translation().z();
      marker.points.push_back(point);
    }

    keyframe_marker_pub_->publish(marker);
  }

  struct SubmapResponseData
  {
    std::string message;
    uint32_t center_keyframe_id{0U};
    std::vector<uint32_t> included_keyframe_ids;
    geometry_msgs::msg::Pose center_local_pose;
    sensor_msgs::msg::PointCloud2 submap;
  };

  SubmapResponseData buildSubmapResponse(std::size_t center_index) const
  {
    const auto & keyframes = keyframe_manager_.keyframes();
    const auto & center_keyframe = keyframes.at(center_index);
    const auto submap_cloud = submap_builder_.buildSubmap(keyframes, center_index);

    const int begin_index =
      std::max(0, static_cast<int>(center_index) - submap_params_.window_size);
    const int end_index =
      std::min(
        static_cast<int>(keyframes.size()) - 1,
        static_cast<int>(center_index) + submap_params_.window_size);

    SubmapResponseData result;
    result.center_keyframe_id = center_keyframe.keyframe_id;
    result.message =
      "built submap around keyframe_id=" + std::to_string(center_keyframe.keyframe_id);
    result.center_local_pose.position.x = center_keyframe.pose_odom_to_keyframe.translation().x();
    result.center_local_pose.position.y = center_keyframe.pose_odom_to_keyframe.translation().y();
    result.center_local_pose.position.z = center_keyframe.pose_odom_to_keyframe.translation().z();

    const Eigen::Quaterniond q(center_keyframe.pose_odom_to_keyframe.rotation());
    result.center_local_pose.orientation.x = q.x();
    result.center_local_pose.orientation.y = q.y();
    result.center_local_pose.orientation.z = q.z();
    result.center_local_pose.orientation.w = q.w();

    for (int idx = begin_index; idx <= end_index; ++idx) {
      result.included_keyframe_ids.push_back(
        keyframes[static_cast<std::size_t>(idx)].keyframe_id);
    }

    pcl::toROSMsg(*submap_cloud, result.submap);
    result.submap.header.stamp = center_keyframe.stamp;
    result.submap.header.frame_id = center_keyframe.frame_id;
    return result;
  }

  static Eigen::Isometry3d odomToIsometry(const nav_msgs::msg::Odometry & odom_msg)
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

  static std::string decisionSummary(const TriggerDecision & decision)
  {
    if (decision.reason_bootstrap) {
      return "bootstrap";
    }

    std::ostringstream oss;
    bool first = true;
    auto append_reason = [&oss, &first](const std::string & reason) {
        if (!first) {
          oss << "|";
        }
        first = false;
        oss << reason;
      };

    if (decision.reason_distance) {
      append_reason("distance");
    }
    if (decision.reason_yaw) {
      append_reason("yaw");
    }
    if (decision.reason_timeout) {
      append_reason("timeout");
    }

    if (first) {
      return "unknown";
    }
    return oss.str();
  }

  uint8_t robot_id_{1U};
  uint32_t next_keyframe_id_{0U};
  double sync_tolerance_sec_{0.12};
  std::string odom_topic_;
  std::string cloud_topic_;
  std::string keyframe_topic_;
  std::string submap_topic_;
    std::string submap_service_name_;
  std::string keyframe_path_topic_;
  std::string keyframe_marker_topic_;
    bool publish_submap_on_request_{true};
  double keyframe_marker_scale_m_{0.35};

  KeyframeTriggerParams trigger_params_;
  SubmapParams submap_params_;
  KeyframeManager keyframe_manager_;
  SubmapBuilder submap_builder_;

  nav_msgs::msg::Path keyframe_path_;
  std::mutex latest_odom_mutex_;
  nav_msgs::msg::Odometry::SharedPtr latest_odom_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<map_consensus_msgs::msg::Keyframe>::SharedPtr keyframe_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr submap_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr keyframe_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr keyframe_marker_pub_;
    rclcpp::Service<map_consensus_msgs::srv::GetSubmap>::SharedPtr submap_service_;
};

}  // namespace keyframe_frontend

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<keyframe_frontend::KeyframeFrontendNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
