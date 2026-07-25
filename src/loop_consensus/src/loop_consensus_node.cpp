#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <map_consensus_msgs/msg/keyframe.hpp>
#include <map_consensus_msgs/msg/loop_candidate.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "loop_consensus/scan_context_manager.hpp"
#include "loop_consensus/types.hpp"

namespace loop_consensus
{

using namespace std::chrono_literals;
constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;

class LoopConsensusNode : public rclcpp::Node
{
public:
  LoopConsensusNode()
  : Node("loop_consensus_node"),
    sc_manager_(loadScanContextParams())
  {
    global_frame_ = declare_parameter<std::string>("global_frame", "map");
    round_period_sec_ = declare_parameter<double>("round_period_sec", 1.0);
    top_k_ = declare_parameter<int>("top_k", 5);
    coarse_candidate_count_ = declare_parameter<int>("coarse_candidate_count", 10);
    intra_robot_exclusion_ = declare_parameter<int>("intra_robot_exclusion", 30);
    marker_z_offset_m_ = declare_parameter<double>("marker_z_offset_m", 0.6);

    keyframe_topics_ = {
      declare_parameter<std::string>("uav1_keyframe_topic", "/uav1/consensus/keyframe"),
      declare_parameter<std::string>("uav2_keyframe_topic", "/uav2/consensus/keyframe"),
      declare_parameter<std::string>("uav3_keyframe_topic", "/uav3/consensus/keyframe")
    };

    candidate_topic_ = declare_parameter<std::string>(
      "candidate_topic", "/map_consensus/loop_candidates");
    marker_topic_ = declare_parameter<std::string>(
      "candidate_marker_topic", "/map_consensus/loop_candidate_markers");

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    const auto qos = rclcpp::SensorDataQoS();
    for (std::size_t idx = 0; idx < keyframe_topics_.size(); ++idx) {
      keyframe_subs_.push_back(
        create_subscription<map_consensus_msgs::msg::Keyframe>(
          keyframe_topics_[idx], qos,
          [this](const map_consensus_msgs::msg::Keyframe::SharedPtr msg) {
            keyframeCallback(msg);
          }));
    }

    candidate_pub_ = create_publisher<map_consensus_msgs::msg::LoopCandidate>(candidate_topic_, 20);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      marker_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());

    timer_ = create_wall_timer(
      std::chrono::duration<double>(round_period_sec_),
      std::bind(&LoopConsensusNode::timerCallback, this));

    RCLCPP_INFO(
      get_logger(),
      "loop_consensus started, topics=[%s, %s, %s], top_k=%d, coarse=%d",
      keyframe_topics_[0].c_str(),
      keyframe_topics_[1].c_str(),
      keyframe_topics_[2].c_str(),
      top_k_,
      coarse_candidate_count_);
  }

private:
  ScanContextParams loadScanContextParams()
  {
    ScanContextParams params;
    params.num_rings = declare_parameter<int>("num_rings", 20);
    params.num_sectors = declare_parameter<int>("num_sectors", 60);
    params.max_radius_m = declare_parameter<double>("max_radius", 40.0);
    params.lidar_height_m = declare_parameter<double>("lidar_height", 2.0);
    params.search_ratio = declare_parameter<double>("search_ratio", 0.1);
    return params;
  }

  void keyframeCallback(const map_consensus_msgs::msg::Keyframe::SharedPtr msg)
  {
    if (msg->robot_id == 0U || msg->robot_id >= pending_queries_.size()) {
      RCLCPP_WARN(
        get_logger(),
        "Ignore keyframe %u because robot_id=%u is invalid.",
        msg->keyframe_id,
        msg->robot_id);
      return;
    }

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(msg->cloud, cloud);
    if (cloud.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "Skip keyframe %u from robot %u because cloud is empty.",
        msg->keyframe_id,
        msg->robot_id);
      return;
    }

    DescriptorEntry entry;
    entry.robot_id = msg->robot_id;
    entry.keyframe_id = msg->keyframe_id;
    entry.graph_version = msg->graph_version;
    entry.robot_sequence = robot_sequences_[msg->robot_id]++;
    entry.stamp = rclcpp::Time(msg->header.stamp);
    entry.frame_id = msg->header.frame_id;
    entry.local_pose = msg->local_pose;
    entry.scan_context = sc_manager_.makeScanContext(cloud);
    entry.ring_key = sc_manager_.makeRingKey(entry.scan_context);
    entry.sector_key = sc_manager_.makeSectorKey(entry.scan_context);

    pending_queries_[msg->robot_id] = entry;
    databases_[msg->robot_id].push_back(std::move(entry));

    RCLCPP_INFO(
      get_logger(),
      "SC stored: robot=%u id=%u seq=%zu database=[%zu,%zu,%zu]",
      msg->robot_id,
      msg->keyframe_id,
      entry.robot_sequence,
      databases_[1].size(),
      databases_[2].size(),
      databases_[3].size());
  }

  void timerCallback()
  {
    const int round = static_cast<int>(round_counter_ % 4U) + 1;
    RCLCPP_INFO(get_logger(), "Polling round %d", round);

    if (round == 1) {
      processRobotQuery(1U, {2U, 3U}, false, round);
    } else if (round == 2) {
      processRobotQuery(2U, {1U, 3U}, false, round);
    } else if (round == 3) {
      processRobotQuery(3U, {1U, 2U}, false, round);
    } else {
      processRobotQuery(1U, {1U}, true, round);
      processRobotQuery(2U, {2U}, true, round);
      processRobotQuery(3U, {3U}, true, round);
    }

    ++round_counter_;
  }

  void processRobotQuery(
    uint8_t query_robot,
    const std::vector<uint8_t> & target_robots,
    bool intra_robot,
    int round)
  {
    if (!pending_queries_[query_robot].has_value()) {
      RCLCPP_INFO(get_logger(), "Round skipped: UAV%u has no keyframe", query_robot);
      publishMarkers({}, round);
      return;
    }

    const auto & query = pending_queries_[query_robot].value();
    auto & last_sequence = intra_robot ? last_intra_sequences_[query_robot] : last_cross_sequences_[query_robot];
    if (last_sequence.has_value() && last_sequence.value() == query.robot_sequence) {
      RCLCPP_INFO(
        get_logger(),
        "Round skipped: UAV%u keyframe %u already queried for %s search",
        query_robot,
        query.keyframe_id,
        intra_robot ? "intra" : "inter");
      publishMarkers({}, round);
      return;
    }

    auto results = findCandidates(query, target_robots, intra_robot);
    logCandidates(query, results);
    publishCandidates(results);
    publishMarkers(results, round);
    last_sequence = query.robot_sequence;
  }

  std::vector<CandidateResult> findCandidates(
    const DescriptorEntry & query,
    const std::vector<uint8_t> & target_robots,
    bool intra_robot) const
  {
    struct CoarseMatch
    {
      const DescriptorEntry * entry{nullptr};
      float ring_distance{0.0F};
    };

    std::vector<CoarseMatch> coarse_matches;

    for (const uint8_t target_robot : target_robots) {
      const auto db_it = databases_.find(target_robot);
      if (db_it == databases_.end()) {
        continue;
      }

      const auto & database = db_it->second;
      for (std::size_t idx = 0; idx < database.size(); ++idx) {
        const auto & candidate = database[idx];
        if (candidate.robot_id == query.robot_id && candidate.keyframe_id == query.keyframe_id) {
          continue;
        }

        if (intra_robot) {
          const std::size_t recent_threshold =
            database.size() > static_cast<std::size_t>(intra_robot_exclusion_)
            ? database.size() - static_cast<std::size_t>(intra_robot_exclusion_)
            : 0U;
          if (idx >= recent_threshold) {
            continue;
          }
        }

        CoarseMatch match;
        match.entry = &candidate;
        match.ring_distance = (query.ring_key - candidate.ring_key).norm();
        coarse_matches.push_back(match);
      }
    }

    if (coarse_matches.empty()) {
      return {};
    }

    std::sort(
      coarse_matches.begin(), coarse_matches.end(),
      [](const CoarseMatch & lhs, const CoarseMatch & rhs) {
        return lhs.ring_distance < rhs.ring_distance;
      });

    if (coarse_matches.size() > static_cast<std::size_t>(coarse_candidate_count_)) {
      coarse_matches.resize(static_cast<std::size_t>(coarse_candidate_count_));
    }

    std::vector<CandidateResult> results;
    results.reserve(coarse_matches.size());
    for (const auto & coarse_match : coarse_matches) {
      const auto [dist, shift] = sc_manager_.distanceBetweenScanContexts(
        query.scan_context,
        coarse_match.entry->scan_context,
        query.sector_key,
        coarse_match.entry->sector_key);

      CandidateResult result;
      result.query = query;
      result.candidate = *coarse_match.entry;
      result.distance = dist;
      result.best_shift = shift;
      result.yaw_initial_rad = sc_manager_.shiftToYawRad(shift);
      result.intra_robot = intra_robot;
      results.push_back(std::move(result));
    }

    std::sort(
      results.begin(), results.end(),
      [](const CandidateResult & lhs, const CandidateResult & rhs) {
        return lhs.distance < rhs.distance;
      });

    if (results.size() > static_cast<std::size_t>(top_k_)) {
      results.resize(static_cast<std::size_t>(top_k_));
    }

    for (std::size_t idx = 0; idx < results.size(); ++idx) {
      results[idx].rank = idx + 1U;
    }

    return results;
  }

  void logCandidates(
    const DescriptorEntry & query,
    const std::vector<CandidateResult> & results) const
  {
    if (results.empty()) {
      RCLCPP_INFO(
        get_logger(),
        "No eligible candidate: robot=%u id=%u",
        query.robot_id,
        query.keyframe_id);
      return;
    }

    std::ostringstream stream;
    for (std::size_t idx = 0; idx < results.size(); ++idx) {
      const auto & result = results[idx];
      if (idx > 0) {
        stream << " | ";
      }

      stream
        << "#" << result.rank
        << " u" << static_cast<int>(result.candidate.robot_id)
        << ":k" << result.candidate.keyframe_id
        << " sc=" << result.distance
        << " yaw_deg=" << result.yaw_initial_rad * kRadToDeg;
    }

    RCLCPP_INFO(
      get_logger(),
      "%s query: u%u:k%u -> %s",
      results.front().intra_robot ? "INTRA" : "INTER",
      query.robot_id,
      query.keyframe_id,
      stream.str().c_str());
  }

  void publishCandidates(const std::vector<CandidateResult> & results)
  {
    for (const auto & result : results) {
      map_consensus_msgs::msg::LoopCandidate msg;
      msg.header.stamp = result.query.stamp;
      msg.header.frame_id = result.query.frame_id;
      msg.rank = static_cast<uint8_t>(result.rank);
      msg.query_robot_id = result.query.robot_id;
      msg.query_keyframe_id = result.query.keyframe_id;
      msg.candidate_robot_id = result.candidate.robot_id;
      msg.candidate_keyframe_id = result.candidate.keyframe_id;
      msg.scan_context_distance = result.distance;
      msg.yaw_initial_rad = result.yaw_initial_rad;
      msg.query_local_pose = result.query.local_pose;
      msg.candidate_local_pose = result.candidate.local_pose;
      msg.graph_version = result.query.graph_version;
      candidate_pub_->publish(msg);
    }
  }

  bool transformPoseToGlobal(
    const DescriptorEntry & entry,
    geometry_msgs::msg::PoseStamped & global_pose)
  {
    geometry_msgs::msg::PoseStamped local_pose_stamped;
    local_pose_stamped.header.stamp = entry.stamp;
    local_pose_stamped.header.frame_id = "odom_uav" + std::to_string(entry.robot_id);
    local_pose_stamped.pose = entry.local_pose;

    try {
      global_pose = tf_buffer_->transform(local_pose_stamped, global_frame_);
      return true;
    } catch (const tf2::TransformException & exception) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Cannot transform %s to %s: %s",
        local_pose_stamped.header.frame_id.c_str(),
        global_frame_.c_str(),
        exception.what());
      return false;
    }
  }

  void publishMarkers(const std::vector<CandidateResult> & results, int round)
  {
    visualization_msgs::msg::MarkerArray marker_array;

    visualization_msgs::msg::Marker delete_all;
    delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(delete_all);

    int marker_id = 0;
    for (const auto & result : results) {
      geometry_msgs::msg::PoseStamped query_global;
      geometry_msgs::msg::PoseStamped candidate_global;
      if (!transformPoseToGlobal(result.query, query_global) ||
        !transformPoseToGlobal(result.candidate, candidate_global))
      {
        continue;
      }

      visualization_msgs::msg::Marker line_marker;
      line_marker.header.stamp = result.query.stamp;
      line_marker.header.frame_id = global_frame_;
      line_marker.ns = "loop_candidate_lines_round_" + std::to_string(round);
      line_marker.id = marker_id++;
      line_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
      line_marker.action = visualization_msgs::msg::Marker::ADD;
      line_marker.pose.orientation.w = 1.0;
      line_marker.scale.x = result.rank == 1U ? 0.10 : 0.05;
      line_marker.color.a = result.rank == 1U ? 1.0F : 0.65F;
      line_marker.color.r = result.intra_robot ? 0.15F : 1.0F;
      line_marker.color.g = result.intra_robot ? 0.85F : 0.25F;
      line_marker.color.b = result.intra_robot ? 0.25F : 0.05F;
      line_marker.lifetime = rclcpp::Duration::from_seconds(round_period_sec_ * 1.2);
      line_marker.points.push_back(query_global.pose.position);
      line_marker.points.push_back(candidate_global.pose.position);
      marker_array.markers.push_back(line_marker);

      geometry_msgs::msg::Point text_anchor;
      text_anchor.x = 0.5 * (query_global.pose.position.x + candidate_global.pose.position.x);
      text_anchor.y = 0.5 * (query_global.pose.position.y + candidate_global.pose.position.y);
      text_anchor.z = std::max(query_global.pose.position.z, candidate_global.pose.position.z);
      text_anchor.z += marker_z_offset_m_ + 0.15 * static_cast<double>(result.rank);

      visualization_msgs::msg::Marker text_marker;
      text_marker.header = line_marker.header;
      text_marker.ns = "loop_candidate_text_round_" + std::to_string(round);
      text_marker.id = marker_id++;
      text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text_marker.action = visualization_msgs::msg::Marker::ADD;
      text_marker.pose.position = text_anchor;
      text_marker.pose.orientation.w = 1.0;
      text_marker.scale.z = 0.35;
      text_marker.color.a = 1.0F;
      text_marker.color.r = line_marker.color.r;
      text_marker.color.g = line_marker.color.g;
      text_marker.color.b = line_marker.color.b;
      text_marker.lifetime = line_marker.lifetime;

      std::ostringstream text;
      text.setf(std::ios::fixed);
      text.precision(3);
      text << "r" << result.rank
           << " "
           << (result.intra_robot ? "intra" : "inter")
           << " "
           << " u" << static_cast<int>(result.query.robot_id) << ":" << result.query.keyframe_id
           << " -> u" << static_cast<int>(result.candidate.robot_id) << ":" << result.candidate.keyframe_id
           << " d=" << result.distance
           << " yaw=" << result.yaw_initial_rad;
      text_marker.text = text.str();
      marker_array.markers.push_back(text_marker);
    }

    marker_pub_->publish(marker_array);
  }

  ScanContextManager sc_manager_;

  std::array<std::string, 3> keyframe_topics_{};
  std::string candidate_topic_;
  std::string marker_topic_;
  std::string global_frame_;

  double round_period_sec_{1.0};
  int top_k_{5};
  int coarse_candidate_count_{10};
  int intra_robot_exclusion_{30};
  double marker_z_offset_m_{0.6};

  std::unordered_map<uint8_t, std::vector<DescriptorEntry>> databases_;
  std::array<std::uint32_t, 4> robot_sequences_{};
  std::array<std::optional<DescriptorEntry>, 4> pending_queries_;
  std::array<std::optional<std::uint32_t>, 4> last_cross_sequences_;
  std::array<std::optional<std::uint32_t>, 4> last_intra_sequences_;
  std::vector<rclcpp::Subscription<map_consensus_msgs::msg::Keyframe>::SharedPtr> keyframe_subs_;
  rclcpp::Publisher<map_consensus_msgs::msg::LoopCandidate>::SharedPtr candidate_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::size_t round_counter_{0U};
};

}  // namespace loop_consensus

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<loop_consensus::LoopConsensusNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
