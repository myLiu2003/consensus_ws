#include <array>
#include <chrono>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_set>

#include <geometry_msgs/msg/pose.hpp>
#include <map_consensus_msgs/msg/loop_candidate.hpp>
#include <map_consensus_msgs/msg/loop_verification.hpp>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "pose_graph_backend/consistency_checker.hpp"
#include "pose_graph_backend/gicp_aligner.hpp"
#include "pose_graph_backend/submap_client.hpp"

namespace pose_graph_backend
{

using namespace std::chrono_literals;

class GicpVerifierNode : public rclcpp::Node
{
public:
  GicpVerifierNode()
  : Node("gicp_verifier_node"),
    submap_response_group_(create_callback_group(rclcpp::CallbackGroupType::Reentrant)),
    submap_client_(this, loadServiceNames(), submap_response_group_),
    gicp_aligner_(loadGicpParams()),
    consistency_checker_(loadConsistencyParams())
  {
    candidate_topic_ = declare_parameter<std::string>(
      "candidate_topic", "/map_consensus/loop_candidates");
    verification_topic_ = declare_parameter<std::string>(
      "verification_topic", "/map_consensus/loop_verifications");
    accepted_topic_ = declare_parameter<std::string>(
      "accepted_topic", "/map_consensus/accepted_loops");
    rejected_topic_ = declare_parameter<std::string>(
      "rejected_topic", "/map_consensus/rejected_loops");

    max_rank_to_verify_ = declare_parameter<int>("max_rank_to_verify", 1);
    sc_distance_threshold_inter_ = declare_parameter<double>("sc_distance_threshold_inter", 0.65);
    sc_distance_threshold_intra_ = declare_parameter<double>("sc_distance_threshold_intra", 0.35);
    request_timeout_sec_ = declare_parameter<double>("request_timeout_sec", 2.0);
    publish_debug_clouds_ = declare_parameter<bool>("publish_debug_clouds", true);
    publish_debug_markers_ = declare_parameter<bool>("publish_debug_markers", true);
    save_jsonl_log_ = declare_parameter<bool>("save_jsonl_log", true);

    verification_pub_ =
      create_publisher<map_consensus_msgs::msg::LoopVerification>(verification_topic_, 20);
    accepted_pub_ =
      create_publisher<map_consensus_msgs::msg::LoopVerification>(accepted_topic_, 20);
    rejected_pub_ =
      create_publisher<map_consensus_msgs::msg::LoopVerification>(rejected_topic_, 20);

    if (publish_debug_clouds_) {
      debug_source_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/map_consensus/gicp_debug/source", 5);
      debug_target_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/map_consensus/gicp_debug/target", 5);
      debug_aligned_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/map_consensus/gicp_debug/aligned", 5);
    }

    if (publish_debug_markers_) {
      debug_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/map_consensus/gicp_debug_markers",
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
    }

    openJsonlLogIfNeeded();

    candidate_sub_ = create_subscription<map_consensus_msgs::msg::LoopCandidate>(
      candidate_topic_,
      rclcpp::QoS(rclcpp::KeepLast(20)).reliable(),
      std::bind(&GicpVerifierNode::candidateCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "gicp_verifier started, candidate_topic=%s verification_topic=%s",
      candidate_topic_.c_str(),
      verification_topic_.c_str());
  }

private:
  std::array<std::string, 4> loadServiceNames()
  {
    std::array<std::string, 4> names{};
    names[1] = declare_parameter<std::string>(
      "uav1_submap_service", "/uav1/consensus/get_submap");
    names[2] = declare_parameter<std::string>(
      "uav2_submap_service", "/uav2/consensus/get_submap");
    names[3] = declare_parameter<std::string>(
      "uav3_submap_service", "/uav3/consensus/get_submap");
    return names;
  }

  GicpParameters loadGicpParams()
  {
    GicpParameters params;
    params.coarse_voxel_size_m = declare_parameter<double>("coarse_voxel_size", 0.4);
    params.fine_voxel_size_m = declare_parameter<double>("fine_voxel_size", 0.2);
    params.max_points = static_cast<std::size_t>(declare_parameter<int>("verifier_max_points", 12000));
    params.coarse_max_correspondence_distance_m =
      declare_parameter<double>("coarse_max_corr_dist", 2.0);
    params.fine_max_correspondence_distance_m =
      declare_parameter<double>("fine_max_corr_dist", 1.0);
    params.coarse_max_iterations = declare_parameter<int>("coarse_max_iter", 30);
    params.fine_max_iterations = declare_parameter<int>("fine_max_iter", 50);
    params.transformation_epsilon = declare_parameter<double>("gicp_trans_eps", 1.0e-3);
    params.euclidean_fitness_epsilon = declare_parameter<double>("gicp_fit_eps", 1.0e-3);
    params.overlap_max_distance_m = declare_parameter<double>("overlap_max_distance", 0.5);
    return params;
  }

  ConsistencyParameters loadConsistencyParams()
  {
    ConsistencyParameters params;
    params.min_submap_points = static_cast<std::size_t>(
      declare_parameter<int>("min_submap_points", 1000));
    params.min_correspondences = static_cast<std::size_t>(
      declare_parameter<int>("min_correspondences", 150));
    params.min_overlap_ratio = declare_parameter<double>("min_overlap_ratio", 0.30);
    params.fitness_threshold_inter = declare_parameter<double>("fitness_threshold_inter", 0.60);
    params.fitness_threshold_intra = declare_parameter<double>("fitness_threshold_intra", 0.35);
    params.max_translation_norm_m = declare_parameter<double>("max_translation_norm", 15.0);
    params.forward_reverse_translation_thresh_m =
      declare_parameter<double>("forward_reverse_translation_thresh", 0.5);
    params.forward_reverse_yaw_thresh_deg =
      declare_parameter<double>("forward_reverse_yaw_thresh_deg", 5.0);
    params.prior_translation_error_thresh_m =
      declare_parameter<double>("prior_translation_error_thresh", 2.0);
    params.prior_yaw_error_deg_thresh =
      declare_parameter<double>("prior_yaw_error_deg_thresh", 15.0);
    params.neighbor_translation_thresh_m =
      declare_parameter<double>("neighbor_translation_thresh", 0.5);
    params.neighbor_yaw_thresh_deg =
      declare_parameter<double>("neighbor_yaw_thresh_deg", 5.0);
    params.neighbor_required_count = static_cast<std::size_t>(
      declare_parameter<int>("neighbor_required_count", 2));
    params.neighbor_max_keyframe_gap = static_cast<uint32_t>(
      declare_parameter<int>("neighbor_max_keyframe_gap", 6));
    return params;
  }

  void openJsonlLogIfNeeded()
  {
    if (!save_jsonl_log_) {
      return;
    }

    const char * ws_env = std::getenv("MAP_CONSENSUS_WS");
    const char * home_env = std::getenv("HOME");
    const std::string workspace = ws_env ? ws_env :
      ((home_env ? std::string(home_env) : std::string("/home/liumengyang")) + "/consensus_lcgo_ws");
    const std::time_t timestamp = std::time(nullptr);
    std::ostringstream stamp;
    std::tm local_tm{};
    localtime_r(&timestamp, &local_tm);
    stamp << std::put_time(&local_tm, "%Y%m%d_%H%M%S");

    const std::filesystem::path log_dir =
      std::filesystem::path(workspace) / "logs" / "pose_graph_backend";
    std::filesystem::create_directories(log_dir);
    jsonl_log_path_ = (log_dir / ("loop_verification_" + stamp.str() + ".jsonl")).string();
    jsonl_stream_.open(jsonl_log_path_, std::ios::out | std::ios::app);

    if (jsonl_stream_.is_open()) {
      RCLCPP_INFO(get_logger(), "verification jsonl log: %s", jsonl_log_path_.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "failed to open jsonl log: %s", jsonl_log_path_.c_str());
    }
  }

  void candidateCallback(const map_consensus_msgs::msg::LoopCandidate::SharedPtr msg)
  {
    const bool intra_robot = msg->query_robot_id == msg->candidate_robot_id;
    if (msg->rank == 0U || static_cast<int>(msg->rank) > max_rank_to_verify_) {
      return;
    }

    const double sc_threshold = intra_robot ? sc_distance_threshold_intra_ : sc_distance_threshold_inter_;
    if (msg->scan_context_distance > sc_threshold) {
      publishEarlyReject(*msg, intra_robot, "sc_distance_too_large");
      return;
    }

    const std::string pair_key = makePairKey(*msg);
    if (!processed_pairs_.insert(pair_key).second) {
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "[GICP_START] pair=u%u:k%u<->u%u:k%u intra=%s rank=%u sc=%.3f",
      msg->query_robot_id,
      msg->query_keyframe_id,
      msg->candidate_robot_id,
      msg->candidate_keyframe_id,
      intra_robot ? "true" : "false",
      msg->rank,
      msg->scan_context_distance);

    SubmapData query_submap;
    SubmapData candidate_submap;
    std::string error_message;

    if (!submap_client_.fetchSubmap(
        msg->query_robot_id,
        msg->query_keyframe_id,
        request_timeout_sec_,
        query_submap,
        error_message))
    {
      publishEarlyReject(*msg, intra_robot, "query_submap_unavailable:" + error_message);
      return;
    }

    if (!submap_client_.fetchSubmap(
        msg->candidate_robot_id,
        msg->candidate_keyframe_id,
        request_timeout_sec_,
        candidate_submap,
        error_message))
    {
      publishEarlyReject(*msg, intra_robot, "candidate_submap_unavailable:" + error_message);
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "[GICP_SUBMAP] pair=u%u:k%u<->u%u:k%u src_pts=%zu tgt_pts=%zu",
      msg->query_robot_id,
      msg->query_keyframe_id,
      msg->candidate_robot_id,
      msg->candidate_keyframe_id,
      query_submap.cloud->size(),
      candidate_submap.cloud->size());

    const auto gicp_result = gicp_aligner_.align(query_submap, candidate_submap, msg->yaw_initial_rad);
    const auto consistency_result = consistency_checker_.evaluate(*msg, gicp_result);
    auto verification_msg = buildVerificationMessage(*msg, intra_robot, gicp_result, consistency_result);

    RCLCPP_INFO(
      get_logger(),
      "[GICP_RESULT] pair=u%u:k%u<->u%u:k%u coarse=%s fine=%s reverse=%s "
      "fitness=%.4f overlap=%.3f corr=%u cycle_trans=%.3f cycle_yaw=%.2f",
      msg->query_robot_id,
      msg->query_keyframe_id,
      msg->candidate_robot_id,
      msg->candidate_keyframe_id,
      gicp_result.coarse_converged ? "true" : "false",
      gicp_result.converged ? "true" : "false",
      gicp_result.reverse_converged ? "true" : "false",
      gicp_result.fitness,
      gicp_result.overlap_ratio,
      gicp_result.correspondence_count,
      gicp_result.forward_reverse_translation_error,
      gicp_result.forward_reverse_yaw_error_deg);

    RCLCPP_INFO(
      get_logger(),
      "[CHECK_RESULT] pair=u%u:k%u<->u%u:k%u accepted=%s reason=%s "
      "prior_trans_err=%.3f prior_yaw_err=%.2f neighbor_trans_err=%.3f "
      "neighbor_yaw_err=%.2f support=%u",
      msg->query_robot_id,
      msg->query_keyframe_id,
      msg->candidate_robot_id,
      msg->candidate_keyframe_id,
      verification_msg.accepted ? "true" : "false",
      verification_msg.reject_reason.c_str(),
      verification_msg.prior_translation_error,
      verification_msg.prior_yaw_error_deg,
      verification_msg.neighbor_translation_error,
      verification_msg.neighbor_yaw_error_deg,
      verification_msg.consistency_support_count);

    verification_pub_->publish(verification_msg);
    if (verification_msg.accepted) {
      accepted_pub_->publish(verification_msg);
    } else {
      rejected_pub_->publish(verification_msg);
    }

    publishDebugArtifacts(*msg, candidate_submap, gicp_result, verification_msg);
    appendJsonl(verification_msg);
  }

  void publishEarlyReject(
    const map_consensus_msgs::msg::LoopCandidate & candidate_msg,
    bool intra_robot,
    const std::string & reason)
  {
    map_consensus_msgs::msg::LoopVerification verification_msg;
    verification_msg.header = candidate_msg.header;
    verification_msg.query_robot_id = candidate_msg.query_robot_id;
    verification_msg.query_keyframe_id = candidate_msg.query_keyframe_id;
    verification_msg.candidate_robot_id = candidate_msg.candidate_robot_id;
    verification_msg.candidate_keyframe_id = candidate_msg.candidate_keyframe_id;
    verification_msg.intra_robot = intra_robot;
    verification_msg.rank = candidate_msg.rank;
    verification_msg.scan_context_distance = candidate_msg.scan_context_distance;
    verification_msg.yaw_initial_rad = candidate_msg.yaw_initial_rad;
    verification_msg.gicp_converged = false;
    verification_msg.accepted = false;
    verification_msg.reject_reason = reason;
    verification_msg.graph_version = candidate_msg.graph_version;

    RCLCPP_INFO(
      get_logger(),
      "[CHECK_RESULT] pair=u%u:k%u<->u%u:k%u accepted=false reason=%s prior_trans_err=0.000 prior_yaw_err=0.00",
      candidate_msg.query_robot_id,
      candidate_msg.query_keyframe_id,
      candidate_msg.candidate_robot_id,
      candidate_msg.candidate_keyframe_id,
      reason.c_str());

    verification_pub_->publish(verification_msg);
    rejected_pub_->publish(verification_msg);
    appendJsonl(verification_msg);
  }

  map_consensus_msgs::msg::LoopVerification buildVerificationMessage(
    const map_consensus_msgs::msg::LoopCandidate & candidate_msg,
    bool intra_robot,
    const GicpResult & gicp_result,
    const ConsistencyResult & consistency_result) const
  {
    map_consensus_msgs::msg::LoopVerification msg;
    msg.header = candidate_msg.header;
    msg.query_robot_id = candidate_msg.query_robot_id;
    msg.query_keyframe_id = candidate_msg.query_keyframe_id;
    msg.candidate_robot_id = candidate_msg.candidate_robot_id;
    msg.candidate_keyframe_id = candidate_msg.candidate_keyframe_id;
    msg.intra_robot = intra_robot;
    msg.rank = candidate_msg.rank;
    msg.scan_context_distance = candidate_msg.scan_context_distance;
    msg.yaw_initial_rad = candidate_msg.yaw_initial_rad;
    msg.gicp_converged = gicp_result.converged;
    msg.gicp_coarse_converged = gicp_result.coarse_converged;
    msg.gicp_reverse_converged = gicp_result.reverse_converged;
    msg.gicp_coarse_fitness = gicp_result.coarse_fitness;
    msg.gicp_fitness = gicp_result.fitness;
    msg.gicp_reverse_fitness = gicp_result.reverse_fitness;
    msg.correspondence_count = gicp_result.correspondence_count;
    msg.overlap_ratio = gicp_result.overlap_ratio;
    msg.forward_reverse_translation_error = gicp_result.forward_reverse_translation_error;
    msg.forward_reverse_yaw_error_deg = gicp_result.forward_reverse_yaw_error_deg;
    msg.gicp_translation_norm = gicp_result.translation_norm;
    msg.gicp_yaw_deg = gicp_result.yaw_deg;
    msg.prior_translation_error = consistency_result.prior_translation_error;
    msg.prior_yaw_error_deg = consistency_result.prior_yaw_error_deg;
    msg.neighbor_translation_error = consistency_result.neighbor_translation_error;
    msg.neighbor_yaw_error_deg = consistency_result.neighbor_yaw_error_deg;
    msg.consistency_support_count = consistency_result.consistency_support_count;
    msg.accepted = consistency_result.accepted;
    msg.reject_reason = consistency_result.reject_reason;
    msg.relative_pose = consistency_result.relative_pose;
    msg.graph_version = candidate_msg.graph_version;
    return msg;
  }

  void publishDebugArtifacts(
    const map_consensus_msgs::msg::LoopCandidate & candidate_msg,
    const SubmapData & target_submap,
    const GicpResult & gicp_result,
    const map_consensus_msgs::msg::LoopVerification & verification_msg)
  {
    if (publish_debug_clouds_ && debug_target_pub_ && debug_source_pub_ && debug_aligned_pub_) {
      sensor_msgs::msg::PointCloud2 target_msg;
      pcl::toROSMsg(*gicp_result.target_cloud, target_msg);
      target_msg.header.stamp = candidate_msg.header.stamp;
      target_msg.header.frame_id = target_submap.frame_id;
      debug_target_pub_->publish(target_msg);

      sensor_msgs::msg::PointCloud2 source_msg;
      pcl::toROSMsg(*gicp_result.source_initial_cloud, source_msg);
      source_msg.header = target_msg.header;
      debug_source_pub_->publish(source_msg);

      sensor_msgs::msg::PointCloud2 aligned_msg;
      pcl::toROSMsg(*gicp_result.aligned_cloud, aligned_msg);
      aligned_msg.header = target_msg.header;
      debug_aligned_pub_->publish(aligned_msg);
    }

    if (publish_debug_markers_ && debug_marker_pub_) {
      visualization_msgs::msg::MarkerArray marker_array;

      visualization_msgs::msg::Marker delete_all;
      delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
      marker_array.markers.push_back(delete_all);

      visualization_msgs::msg::Marker text_marker;
      text_marker.header.stamp = candidate_msg.header.stamp;
      text_marker.header.frame_id = target_submap.frame_id;
      text_marker.ns = "gicp_verification";
      text_marker.id = 0;
      text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text_marker.action = visualization_msgs::msg::Marker::ADD;
      text_marker.pose.orientation.w = 1.0;
      text_marker.pose.position.z = 1.0;
      text_marker.scale.z = 0.5;
      text_marker.color.a = 1.0F;
      text_marker.color.r = verification_msg.accepted ? 0.1F : 1.0F;
      text_marker.color.g = verification_msg.accepted ? 0.9F : 0.2F;
      text_marker.color.b = 0.2F;
      text_marker.lifetime = rclcpp::Duration::from_seconds(2.0);

      std::ostringstream text;
      text.setf(std::ios::fixed);
      text.precision(3);
      text << (verification_msg.accepted ? "ACCEPT" : "REJECT")
           << " u" << static_cast<int>(verification_msg.query_robot_id) << ":" << verification_msg.query_keyframe_id
           << " -> u" << static_cast<int>(verification_msg.candidate_robot_id) << ":" << verification_msg.candidate_keyframe_id
           << " fit=" << verification_msg.gicp_fitness
           << " overlap=" << verification_msg.overlap_ratio
           << " support=" << verification_msg.consistency_support_count
           << " reason=" << verification_msg.reject_reason;
      text_marker.text = text.str();
      marker_array.markers.push_back(text_marker);
      debug_marker_pub_->publish(marker_array);
    }
  }

  void appendJsonl(const map_consensus_msgs::msg::LoopVerification & msg)
  {
    if (!jsonl_stream_.is_open()) {
      return;
    }

    std::scoped_lock<std::mutex> lock(jsonl_mutex_);
    jsonl_stream_
      << "{"
      << "\"stamp_sec\":" << rclcpp::Time(msg.header.stamp).seconds() << ","
      << "\"query_robot_id\":" << static_cast<int>(msg.query_robot_id) << ","
      << "\"query_keyframe_id\":" << msg.query_keyframe_id << ","
      << "\"candidate_robot_id\":" << static_cast<int>(msg.candidate_robot_id) << ","
      << "\"candidate_keyframe_id\":" << msg.candidate_keyframe_id << ","
      << "\"intra_robot\":" << (msg.intra_robot ? "true" : "false") << ","
      << "\"rank\":" << static_cast<int>(msg.rank) << ","
      << "\"scan_context_distance\":" << msg.scan_context_distance << ","
      << "\"gicp_converged\":" << (msg.gicp_converged ? "true" : "false") << ","
      << "\"gicp_coarse_converged\":" << (msg.gicp_coarse_converged ? "true" : "false") << ","
      << "\"gicp_reverse_converged\":" << (msg.gicp_reverse_converged ? "true" : "false") << ","
      << "\"gicp_coarse_fitness\":" << msg.gicp_coarse_fitness << ","
      << "\"gicp_fitness\":" << msg.gicp_fitness << ","
      << "\"gicp_reverse_fitness\":" << msg.gicp_reverse_fitness << ","
      << "\"correspondence_count\":" << msg.correspondence_count << ","
      << "\"overlap_ratio\":" << msg.overlap_ratio << ","
      << "\"forward_reverse_translation_error\":" << msg.forward_reverse_translation_error << ","
      << "\"forward_reverse_yaw_error_deg\":" << msg.forward_reverse_yaw_error_deg << ","
      << "\"gicp_translation_norm\":" << msg.gicp_translation_norm << ","
      << "\"gicp_yaw_deg\":" << msg.gicp_yaw_deg << ","
      << "\"prior_translation_error\":" << msg.prior_translation_error << ","
      << "\"prior_yaw_error_deg\":" << msg.prior_yaw_error_deg << ","
      << "\"neighbor_translation_error\":" << msg.neighbor_translation_error << ","
      << "\"neighbor_yaw_error_deg\":" << msg.neighbor_yaw_error_deg << ","
      << "\"consistency_support_count\":" << msg.consistency_support_count << ","
      << "\"accepted\":" << (msg.accepted ? "true" : "false") << ","
      << "\"reject_reason\":\"" << msg.reject_reason << "\""
      << "}\n";
    jsonl_stream_.flush();
  }

  static std::string makePairKey(const map_consensus_msgs::msg::LoopCandidate & msg)
  {
    std::ostringstream key;
    key << static_cast<int>(msg.query_robot_id) << ":" << msg.query_keyframe_id
        << "->"
        << static_cast<int>(msg.candidate_robot_id) << ":" << msg.candidate_keyframe_id;
    return key.str();
  }

  std::string candidate_topic_;
  std::string verification_topic_;
  std::string accepted_topic_;
  std::string rejected_topic_;

  int max_rank_to_verify_{1};
  double sc_distance_threshold_inter_{0.65};
  double sc_distance_threshold_intra_{0.35};
  double request_timeout_sec_{2.0};
  bool publish_debug_clouds_{true};
  bool publish_debug_markers_{true};
  bool save_jsonl_log_{true};

  std::unordered_set<std::string> processed_pairs_;
  std::string jsonl_log_path_;
  std::ofstream jsonl_stream_;
  std::mutex jsonl_mutex_;

  // Service responses must not share the mutually exclusive candidate callback group.
  rclcpp::CallbackGroup::SharedPtr submap_response_group_;
  SubmapClient submap_client_;
  GicpAligner gicp_aligner_;
  ConsistencyChecker consistency_checker_;

  rclcpp::Subscription<map_consensus_msgs::msg::LoopCandidate>::SharedPtr candidate_sub_;
  rclcpp::Publisher<map_consensus_msgs::msg::LoopVerification>::SharedPtr verification_pub_;
  rclcpp::Publisher<map_consensus_msgs::msg::LoopVerification>::SharedPtr accepted_pub_;
  rclcpp::Publisher<map_consensus_msgs::msg::LoopVerification>::SharedPtr rejected_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_source_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_target_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_aligned_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_marker_pub_;
};

}  // namespace pose_graph_backend

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<pose_graph_backend::GicpVerifierNode>();
  // One thread may wait in candidateCallback while the other delivers service responses.
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
