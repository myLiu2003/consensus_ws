#include <gtest/gtest.h>

#include <map_consensus_msgs/msg/loop_candidate.hpp>

#include "pose_graph_backend/consistency_checker.hpp"
#include "pose_graph_backend/types.hpp"

namespace pose_graph_backend
{
namespace
{

GicpResult makePassingGicpResult()
{
  GicpResult result;
  result.coarse_converged = true;
  result.converged = true;
  result.reverse_converged = true;
  result.fitness = 0.1F;
  result.reverse_fitness = 0.1F;
  result.correspondence_count = 500U;
  result.overlap_ratio = 0.7F;
  result.forward_reverse_translation_error = 0.05F;
  result.forward_reverse_yaw_error_deg = 0.5F;
  result.source_cloud->resize(1500U);
  result.target_cloud->resize(1500U);
  return result;
}

map_consensus_msgs::msg::LoopCandidate makeCandidate(uint32_t query_id, uint32_t candidate_id)
{
  map_consensus_msgs::msg::LoopCandidate candidate;
  candidate.query_robot_id = 1U;
  candidate.candidate_robot_id = 2U;
  candidate.query_keyframe_id = query_id;
  candidate.candidate_keyframe_id = candidate_id;
  candidate.query_local_pose.orientation.w = 1.0;
  candidate.candidate_local_pose.orientation.w = 1.0;
  return candidate;
}

TEST(ConsistencyChecker, RejectsInsufficientOverlap)
{
  ConsistencyChecker checker(ConsistencyParameters{});
  auto gicp = makePassingGicpResult();
  gicp.overlap_ratio = 0.2F;
  const auto result = checker.evaluate(makeCandidate(10U, 20U), gicp);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.reject_reason, "overlap_too_small");
}

TEST(ConsistencyChecker, RequiresTwoAdjacentInterRobotResults)
{
  ConsistencyChecker checker(ConsistencyParameters{});
  const auto gicp = makePassingGicpResult();

  const auto first = checker.evaluate(makeCandidate(10U, 20U), gicp);
  EXPECT_FALSE(first.accepted);
  EXPECT_EQ(first.reject_reason, "awaiting_neighbor_consistency");
  EXPECT_EQ(first.consistency_support_count, 1U);

  const auto second = checker.evaluate(makeCandidate(11U, 21U), gicp);
  EXPECT_TRUE(second.accepted);
  EXPECT_EQ(second.reject_reason, "accepted");
  EXPECT_EQ(second.consistency_support_count, 2U);
}

TEST(ConsistencyChecker, ResetsOnInconsistentNeighborTransform)
{
  ConsistencyChecker checker(ConsistencyParameters{});
  const auto first_gicp = makePassingGicpResult();
  auto second_gicp = makePassingGicpResult();
  second_gicp.final_transform(0, 3) = 1.0F;

  checker.evaluate(makeCandidate(10U, 20U), first_gicp);
  const auto second = checker.evaluate(makeCandidate(11U, 21U), second_gicp);
  EXPECT_FALSE(second.accepted);
  EXPECT_EQ(second.reject_reason, "neighbor_transform_inconsistent");
  EXPECT_EQ(second.consistency_support_count, 1U);
}

}  // namespace
}  // namespace pose_graph_backend
