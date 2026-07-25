#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "map_consensus_msgs/msg/keyframe.hpp"
#include "map_consensus_msgs/msg/loop_candidate.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"

class LoopConsensusNode : public rclcpp::Node
{
public:
  using Keyframe = map_consensus_msgs::msg::Keyframe;
  using LoopCandidate = map_consensus_msgs::msg::LoopCandidate;
  using Marker = visualization_msgs::msg::Marker;

  LoopConsensusNode()
  : Node("loop_consensus_node")
  {
    global_frame_ = this->declare_parameter<std::string>(
      "global_frame", "map");

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(
      this->get_clock());

    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(
      *tf_buffer_);

    candidate_publisher_ = this->create_publisher<LoopCandidate>(
      "/map_consensus/loop_candidates",
      rclcpp::QoS(rclcpp::KeepLast(50)).reliable());

    marker_publisher_ = this->create_publisher<Marker>(
      "/map_consensus/loop_candidate_markers",
      rclcpp::QoS(rclcpp::KeepLast(50)).reliable());

    RCLCPP_INFO(
      this->get_logger(),
      "Publishing loop candidates on /map_consensus/loop_candidates");

    RCLCPP_INFO(
      this->get_logger(),
      "Publishing candidate markers on "
      "/map_consensus/loop_candidate_markers, global_frame=%s",
      global_frame_.c_str());

    for (int robot_id = 1; robot_id <= 3; ++robot_id) {
      const std::string topic =
        "/uav" + std::to_string(robot_id) + "/consensus/keyframe";

      auto subscription = this->create_subscription<Keyframe>(
        topic,
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
        [this, topic](const Keyframe::SharedPtr msg) {
          this->keyframe_callback(msg, topic);
        });

      subscriptions_.push_back(subscription);

      RCLCPP_INFO(
        this->get_logger(),
        "Subscribed to %s",
        topic.c_str());
    }

    polling_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&LoopConsensusNode::polling_callback, this));
  }

private:
  static constexpr std::size_t kNumRings = 20;
  static constexpr std::size_t kNumSectors = 60;
  static constexpr std::size_t kTopK = 5;
  static constexpr std::uint32_t kIntraRobotExclusion = 30;
  static constexpr float kMaxRadius = 40.0F;
  static constexpr float kLidarHeight = 2.0F;
  static constexpr float kPi = 3.14159265358979323846F;

  struct DescriptorResult
  {
    std::vector<float> descriptor;
    std::size_t valid_points;
    std::size_t occupied_bins;
  };

  struct DescriptorEntry
  {
    std::uint8_t robot_id;
    std::uint32_t keyframe_id;
    std::uint32_t robot_sequence;
    Keyframe::_header_type header;
    Keyframe::_local_pose_type local_pose;
    std::uint32_t graph_version;
    std::vector<float> descriptor;
    std::vector<float> ring_key;
  };

  struct Candidate
  {
    std::size_t entry_index;
    float ring_distance;
    float sc_distance;
    int sector_shift;
  };

  struct PendingQuery
  {
    Keyframe::SharedPtr keyframe;
    std::uint32_t robot_sequence;
    std::vector<float> descriptor;
    std::vector<float> ring_key;
  };

  DescriptorResult make_scan_context(
    const sensor_msgs::msg::PointCloud2 & cloud) const
  {
    const float empty_value =
      -std::numeric_limits<float>::infinity();

    std::vector<float> descriptor(
      kNumRings * kNumSectors,
      empty_value);

    std::size_t valid_points = 0;

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      const float x = *iter_x;
      const float y = *iter_y;
      const float z = *iter_z;

      if (!std::isfinite(x) ||
        !std::isfinite(y) ||
        !std::isfinite(z))
      {
        continue;
      }

      const float radius = std::hypot(x, y);

      if (radius < 0.1F || radius > kMaxRadius) {
        continue;
      }

      float angle = std::atan2(y, x);

      if (angle < 0.0F) {
        angle += 2.0F * kPi;
      }

      std::size_t ring_index = static_cast<std::size_t>(
        radius / kMaxRadius * static_cast<float>(kNumRings));

      std::size_t sector_index = static_cast<std::size_t>(
        angle / (2.0F * kPi) * static_cast<float>(kNumSectors));

      ring_index = std::min(ring_index, kNumRings - 1);
      sector_index = std::min(sector_index, kNumSectors - 1);

      const std::size_t descriptor_index =
        ring_index * kNumSectors + sector_index;

      const float height_value = z + kLidarHeight;

      descriptor[descriptor_index] = std::max(
        descriptor[descriptor_index],
        height_value);

      ++valid_points;
    }

    std::size_t occupied_bins = 0;

    for (float & value : descriptor) {
      if (!std::isfinite(value)) {
        value = 0.0F;
      } else {
        ++occupied_bins;
      }
    }

    return {
      std::move(descriptor),
      valid_points,
      occupied_bins
    };
  }

  std::vector<float> make_ring_key(
    const std::vector<float> & descriptor) const
  {
    std::vector<float> ring_key(kNumRings, 0.0F);

    for (std::size_t ring = 0; ring < kNumRings; ++ring) {
      float sum = 0.0F;

      for (std::size_t sector = 0;
        sector < kNumSectors;
        ++sector)
      {
        sum += descriptor[ring * kNumSectors + sector];
      }

      ring_key[ring] =
        sum / static_cast<float>(kNumSectors);
    }

    return ring_key;
  }

  float ring_key_distance(
    const std::vector<float> & first,
    const std::vector<float> & second) const
  {
    float squared_sum = 0.0F;

    for (std::size_t index = 0; index < first.size(); ++index) {
      const float difference = first[index] - second[index];
      squared_sum += difference * difference;
    }

    return std::sqrt(squared_sum);
  }

  std::pair<float, int> scan_context_distance(
    const std::vector<float> & query,
    const std::vector<float> & candidate) const
  {
    float best_distance = std::numeric_limits<float>::infinity();
    int best_shift = 0;

    for (std::size_t shift = 0; shift < kNumSectors; ++shift) {
      float similarity_sum = 0.0F;
      std::size_t compared_sectors = 0;

      for (std::size_t sector = 0;
        sector < kNumSectors;
        ++sector)
      {
        const std::size_t shifted_sector =
          (sector + shift) % kNumSectors;

        float dot_product = 0.0F;
        float query_norm = 0.0F;
        float candidate_norm = 0.0F;

        for (std::size_t ring = 0; ring < kNumRings; ++ring) {
          const float query_value =
            query[ring * kNumSectors + sector];

          const float candidate_value =
            candidate[ring * kNumSectors + shifted_sector];

          dot_product += query_value * candidate_value;
          query_norm += query_value * query_value;
          candidate_norm += candidate_value * candidate_value;
        }

        if (query_norm < 1.0e-6F ||
          candidate_norm < 1.0e-6F)
        {
          continue;
        }

        float similarity =
          dot_product / std::sqrt(query_norm * candidate_norm);

        similarity = std::clamp(similarity, -1.0F, 1.0F);
        similarity_sum += similarity;
        ++compared_sectors;
      }

      if (compared_sectors == 0) {
        continue;
      }

      const float distance =
        1.0F -
        similarity_sum / static_cast<float>(compared_sectors);

      if (distance < best_distance) {
        best_distance = distance;
        best_shift = static_cast<int>(shift);
      }
    }

    return {best_distance, best_shift};
  }

  std::vector<Candidate> find_candidates(
    std::uint8_t query_robot_id,
    std::uint32_t query_sequence,
    const std::vector<float> & query_descriptor,
    const std::vector<float> & query_ring_key,
    bool cross_robot_search) const
  {
    std::vector<Candidate> candidates;

    for (std::size_t index = 0; index < database_.size(); ++index) {
      const auto & entry = database_[index];

      if (cross_robot_search) {
        if (entry.robot_id == query_robot_id) {
          continue;
        }
      } else {
        if (entry.robot_id != query_robot_id) {
          continue;
        }

        if (query_sequence <=
          entry.robot_sequence + kIntraRobotExclusion)
        {
          continue;
        }
      }

      candidates.push_back({
        index,
        ring_key_distance(query_ring_key, entry.ring_key),
        1.0F,
        0
      });
    }

    std::sort(
      candidates.begin(),
      candidates.end(),
      [](const Candidate & first, const Candidate & second) {
        return first.ring_distance < second.ring_distance;
      });

    if (candidates.size() > kTopK) {
      candidates.resize(kTopK);
    }

    for (auto & candidate : candidates) {
      const auto result = scan_context_distance(
        query_descriptor,
        database_[candidate.entry_index].descriptor);

      candidate.sc_distance = result.first;
      candidate.sector_shift = result.second;
    }

    std::sort(
      candidates.begin(),
      candidates.end(),
      [](const Candidate & first, const Candidate & second) {
        return first.sc_distance < second.sc_distance;
      });

    return candidates;
  }

  float shift_to_yaw_degrees(int sector_shift) const
  {
    float yaw =
      static_cast<float>(sector_shift) *
      360.0F /
      static_cast<float>(kNumSectors);

    if (yaw > 180.0F) {
      yaw -= 360.0F;
    }

    return yaw;
  }

  float shift_to_yaw_radians(int sector_shift) const
  {
    float yaw =
      static_cast<float>(sector_shift) *
      2.0F * kPi /
      static_cast<float>(kNumSectors);

    if (yaw > kPi) {
      yaw -= 2.0F * kPi;
    }

    return yaw;
  }

  void log_candidates(
    const Keyframe & query,
    const std::vector<Candidate> & candidates) const
  {
    if (candidates.empty()) {
      RCLCPP_INFO(
        this->get_logger(),
        "No eligible candidate: robot=%u id=%u database_size=%lu",
        static_cast<unsigned int>(query.robot_id),
        static_cast<unsigned int>(query.keyframe_id),
        static_cast<unsigned long>(database_.size()));

      return;
    }

    std::ostringstream stream;

    for (std::size_t rank = 0; rank < candidates.size(); ++rank) {
      const auto & candidate = candidates[rank];
      const auto & entry = database_[candidate.entry_index];

      if (rank > 0) {
        stream << " | ";
      }

      stream
        << "#" << rank + 1
        << " r" << static_cast<unsigned int>(entry.robot_id)
        << ":k" << entry.keyframe_id
        << " sc=" << candidate.sc_distance
        << " yaw_deg=" << shift_to_yaw_degrees(candidate.sector_shift);
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Top-%lu query=r%u:k%u -> %s",
      static_cast<unsigned long>(candidates.size()),
      static_cast<unsigned int>(query.robot_id),
      static_cast<unsigned int>(query.keyframe_id),
      stream.str().c_str());
  }

  bool transform_pose_to_global(
    std::uint8_t robot_id,
    const Keyframe::_local_pose_type & local_pose,
    const Keyframe::_header_type & source_header,
    geometry_msgs::msg::PoseStamped & global_pose)
  {
    geometry_msgs::msg::PoseStamped local_pose_stamped;

    local_pose_stamped.header.stamp = source_header.stamp;
    local_pose_stamped.header.frame_id =
      "odom_uav" +
      std::to_string(static_cast<unsigned int>(robot_id));
    local_pose_stamped.pose = local_pose;

    try {
      global_pose = tf_buffer_->transform(
        local_pose_stamped,
        global_frame_);

      return true;
    } catch (const tf2::TransformException & exception) {
      RCLCPP_WARN(
        this->get_logger(),
        "Cannot transform %s to %s: %s",
        local_pose_stamped.header.frame_id.c_str(),
        global_frame_.c_str(),
        exception.what());

      return false;
    }
  }

  void publish_candidate_marker(
    const Keyframe & query,
    const DescriptorEntry & candidate,
    std::size_t rank,
    float score)
  {
    geometry_msgs::msg::PoseStamped query_global;
    geometry_msgs::msg::PoseStamped candidate_global;

    if (!transform_pose_to_global(
        query.robot_id,
        query.local_pose,
        query.header,
        query_global))
    {
      return;
    }

    if (!transform_pose_to_global(
        candidate.robot_id,
        candidate.local_pose,
        candidate.header,
        candidate_global))
    {
      return;
    }

    Marker marker;
    marker.header.frame_id = global_frame_;
    marker.header.stamp = query.header.stamp;
    marker.ns = "loop_candidates";
    marker.id = marker_id_++;
    marker.type = Marker::LINE_LIST;
    marker.action = Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = rank == 0 ? 0.10 : 0.05;
    marker.color.r = 1.0F;
    marker.color.g = rank == 0 ? 0.1F : 0.7F;
    marker.color.b = 0.0F;
    marker.color.a = rank == 0 ? 1.0F : 0.55F;
    marker.points.push_back(query_global.pose.position);
    marker.points.push_back(candidate_global.pose.position);
    marker.lifetime.sec = 3;
    marker.lifetime.nanosec = 0;
    marker.text =
      "rank=" + std::to_string(rank + 1) +
      " score=" + std::to_string(score);

    marker_publisher_->publish(marker);
  }

  void publish_candidates(
    const Keyframe & query,
    const std::vector<Candidate> & candidates)
  {
    for (std::size_t index = 0; index < candidates.size(); ++index) {
      const auto & candidate = candidates[index];
      const auto & entry = database_[candidate.entry_index];

      LoopCandidate output;
      output.header = query.header;
      output.rank = static_cast<std::uint8_t>(index + 1);
      output.query_robot_id = query.robot_id;
      output.query_keyframe_id = query.keyframe_id;
      output.candidate_robot_id = entry.robot_id;
      output.candidate_keyframe_id = entry.keyframe_id;
      output.scan_context_distance = candidate.sc_distance;
      output.yaw_initial_rad =
        shift_to_yaw_radians(candidate.sector_shift);
      output.query_local_pose = query.local_pose;
      output.candidate_local_pose = entry.local_pose;
      output.graph_version = query.graph_version;

      candidate_publisher_->publish(output);

      publish_candidate_marker(
        query,
        entry,
        index,
        candidate.sc_distance);
    }
  }

  void process_robot_query(
    std::size_t robot_index,
    bool cross_robot_search)
  {
    if (robot_index == 0 || robot_index >= pending_queries_.size()) {
      return;
    }

    if (!pending_queries_[robot_index].has_value()) {
      RCLCPP_INFO(
        this->get_logger(),
        "Round skipped: UAV%lu has no keyframe",
        static_cast<unsigned long>(robot_index));

      return;
    }

    const auto & pending = pending_queries_[robot_index].value();

    auto & last_sequence =
      cross_robot_search ?
      last_cross_sequences_[robot_index] :
      last_intra_sequences_[robot_index];

    if (last_sequence.has_value() &&
      last_sequence.value() == pending.robot_sequence)
    {
      RCLCPP_INFO(
        this->get_logger(),
        "Round skipped: UAV%lu keyframe already queried",
        static_cast<unsigned long>(robot_index));

      return;
    }

    const auto candidates = find_candidates(
      pending.keyframe->robot_id,
      pending.robot_sequence,
      pending.descriptor,
      pending.ring_key,
      cross_robot_search);

    RCLCPP_INFO(
      this->get_logger(),
      "%s query: UAV%lu keyframe=%u database=%lu",
      cross_robot_search ? "INTER" : "INTRA",
      static_cast<unsigned long>(robot_index),
      static_cast<unsigned int>(pending.keyframe->keyframe_id),
      static_cast<unsigned long>(database_.size()));

    log_candidates(*pending.keyframe, candidates);
    publish_candidates(*pending.keyframe, candidates);
    last_sequence = pending.robot_sequence;
  }

  void polling_callback()
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Polling round %u",
      static_cast<unsigned int>(polling_round_));

    switch (polling_round_) {
      case 1:
        process_robot_query(1, true);
        break;
      case 2:
        process_robot_query(2, true);
        break;
      case 3:
        process_robot_query(3, true);
        break;
      case 4:
        process_robot_query(1, false);
        process_robot_query(2, false);
        process_robot_query(3, false);
        break;
      default:
        polling_round_ = 1;
        break;
    }

    polling_round_ = polling_round_ % 4 + 1;
  }

  void keyframe_callback(
    const Keyframe::SharedPtr msg,
    const std::string & topic)
  {
    try {
      const std::size_t robot_index =
        static_cast<std::size_t>(msg->robot_id);

      if (robot_index == 0 ||
        robot_index >= robot_sequences_.size())
      {
        RCLCPP_WARN(
          this->get_logger(),
          "Ignoring invalid robot_id=%u",
          static_cast<unsigned int>(msg->robot_id));

        return;
      }

      const std::uint32_t robot_sequence =
        robot_sequences_[robot_index]++;

      auto descriptor_result = make_scan_context(msg->cloud);
      auto ring_key = make_ring_key(descriptor_result.descriptor);

      RCLCPP_INFO(
        this->get_logger(),
        "SC stored: topic=%s robot=%u id=%u "
        "valid=%lu occupied=%lu/1200",
        topic.c_str(),
        static_cast<unsigned int>(msg->robot_id),
        static_cast<unsigned int>(msg->keyframe_id),
        static_cast<unsigned long>(descriptor_result.valid_points),
        static_cast<unsigned long>(descriptor_result.occupied_bins));

      PendingQuery pending;
      pending.keyframe = msg;
      pending.robot_sequence = robot_sequence;
      pending.descriptor = descriptor_result.descriptor;
      pending.ring_key = ring_key;
      pending_queries_[robot_index] = std::move(pending);

      DescriptorEntry new_entry;
      new_entry.robot_id = msg->robot_id;
      new_entry.keyframe_id = msg->keyframe_id;
      new_entry.robot_sequence = robot_sequence;
      new_entry.header = msg->header;
      new_entry.local_pose = msg->local_pose;
      new_entry.graph_version = msg->graph_version;
      new_entry.descriptor = std::move(descriptor_result.descriptor);
      new_entry.ring_key = std::move(ring_key);

      database_.push_back(std::move(new_entry));
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Scan Context processing failed: %s",
        exception.what());
    }
  }

  std::vector<rclcpp::Subscription<Keyframe>::SharedPtr>
    subscriptions_;

  rclcpp::Publisher<LoopCandidate>::SharedPtr
    candidate_publisher_;

  rclcpp::Publisher<Marker>::SharedPtr
    marker_publisher_;

  std::string global_frame_;

  std::unique_ptr<tf2_ros::Buffer>
    tf_buffer_;

  std::shared_ptr<tf2_ros::TransformListener>
    tf_listener_;

  rclcpp::TimerBase::SharedPtr
    polling_timer_;

  std::int32_t marker_id_{0};
  std::uint8_t polling_round_{1};

  std::vector<DescriptorEntry>
    database_;

  std::array<std::uint32_t, 4>
    robot_sequences_{};

  std::array<std::optional<PendingQuery>, 4>
    pending_queries_;

  std::array<std::optional<std::uint32_t>, 4>
    last_cross_sequences_;

  std::array<std::optional<std::uint32_t>, 4>
    last_intra_sequences_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LoopConsensusNode>());
  rclcpp::shutdown();
  return 0;
}
