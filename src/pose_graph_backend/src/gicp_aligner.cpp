#include "pose_graph_backend/gicp_aligner.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <utility>
#include <vector>

#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/gicp.h>

namespace pose_graph_backend
{

namespace
{

constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;

struct DirectionResult
{
  bool converged{false};
  float fitness{0.0F};
  Eigen::Matrix4f transform{Eigen::Matrix4f::Identity()};
  PointCloudPtr aligned{new PointCloud()};
};

PointCloudPtr removeNonFinitePoints(const PointCloudPtr & input_cloud)
{
  PointCloudPtr finite_cloud(new PointCloud());
  finite_cloud->reserve(input_cloud->size());
  for (const auto & point : input_cloud->points) {
    if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
      finite_cloud->push_back(point);
    }
  }
  finite_cloud->width = static_cast<std::uint32_t>(finite_cloud->size());
  finite_cloud->height = 1U;
  finite_cloud->is_dense = true;
  return finite_cloud;
}

DirectionResult runGicp(
  const PointCloudPtr & source,
  const PointCloudPtr & target,
  const Eigen::Matrix4f & initial_guess,
  double max_correspondence_distance_m,
  int max_iterations,
  double transformation_epsilon,
  double euclidean_fitness_epsilon)
{
  DirectionResult result;
  if (!source || !target || source->empty() || target->empty()) {
    return result;
  }

  pcl::GeneralizedIterativeClosestPoint<PointType, PointType> gicp;
  gicp.setMaxCorrespondenceDistance(max_correspondence_distance_m);
  gicp.setMaximumIterations(max_iterations);
  gicp.setTransformationEpsilon(transformation_epsilon);
  gicp.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon);
  gicp.setInputSource(source);
  gicp.setInputTarget(target);
  gicp.align(*result.aligned, initial_guess);
  result.converged = gicp.hasConverged();
  result.fitness = static_cast<float>(gicp.getFitnessScore());
  result.transform = gicp.getFinalTransformation();
  return result;
}

std::pair<uint32_t, float> calculateSymmetricOverlap(
  const PointCloudPtr & aligned_source,
  const PointCloudPtr & target,
  double max_distance_m)
{
  if (!aligned_source || !target || aligned_source->empty() || target->empty()) {
    return {0U, 0.0F};
  }

  const float max_distance_sq = static_cast<float>(max_distance_m * max_distance_m);
  auto count_near = [max_distance_sq](const PointCloudPtr & query, const PointCloudPtr & reference) {
      pcl::KdTreeFLANN<PointType> tree;
      tree.setInputCloud(reference);
      std::vector<int> indices(1);
      std::vector<float> distances(1);
      uint32_t count = 0U;
      for (const auto & point : query->points) {
        if (tree.nearestKSearch(point, 1, indices, distances) > 0 &&
          distances[0] <= max_distance_sq)
        {
          ++count;
        }
      }
      return count;
    };

  const uint32_t source_matches = count_near(aligned_source, target);
  const uint32_t target_matches = count_near(target, aligned_source);
  const float source_ratio = static_cast<float>(source_matches) /
    static_cast<float>(aligned_source->size());
  const float target_ratio = static_cast<float>(target_matches) /
    static_cast<float>(target->size());
  return {std::min(source_matches, target_matches), std::min(source_ratio, target_ratio)};
}

}  // namespace

GicpAligner::GicpAligner(const GicpParameters & params)
: params_(params)
{
}

GicpResult GicpAligner::align(
  const SubmapData & source_submap,
  const SubmapData & target_submap,
  float yaw_initial_rad) const
{
  GicpResult result;
  const auto coarse_source = preprocess(source_submap.cloud, params_.coarse_voxel_size_m);
  const auto coarse_target = preprocess(target_submap.cloud, params_.coarse_voxel_size_m);
  result.source_cloud = preprocess(source_submap.cloud, params_.fine_voxel_size_m);
  result.target_cloud = preprocess(target_submap.cloud, params_.fine_voxel_size_m);

  if (coarse_source->empty() || coarse_target->empty() || result.source_cloud->empty() ||
    result.target_cloud->empty() || !std::isfinite(yaw_initial_rad))
  {
    return result;
  }

  Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity();
  const float cos_yaw = std::cos(yaw_initial_rad);
  const float sin_yaw = std::sin(yaw_initial_rad);
  initial_guess(0, 0) = cos_yaw;
  initial_guess(0, 1) = -sin_yaw;
  initial_guess(1, 0) = sin_yaw;
  initial_guess(1, 1) = cos_yaw;

  pcl::transformPointCloud(*result.source_cloud, *result.source_initial_cloud, initial_guess);
  result.source_initial_cloud = removeNonFinitePoints(result.source_initial_cloud);

  const auto forward_coarse = runGicp(
    coarse_source, coarse_target, initial_guess,
    params_.coarse_max_correspondence_distance_m, params_.coarse_max_iterations,
    params_.transformation_epsilon, params_.euclidean_fitness_epsilon);
  result.coarse_converged = forward_coarse.converged;
  result.coarse_fitness = forward_coarse.fitness;
  if (!forward_coarse.converged) {
    return result;
  }

  const auto forward_fine = runGicp(
    result.source_cloud, result.target_cloud, forward_coarse.transform,
    params_.fine_max_correspondence_distance_m, params_.fine_max_iterations,
    params_.transformation_epsilon, params_.euclidean_fitness_epsilon);
  result.converged = forward_fine.converged;
  result.fitness = forward_fine.fitness;
  result.final_transform = forward_fine.transform;
  result.aligned_cloud = forward_fine.aligned;

  Eigen::Matrix4f reverse_initial_guess = Eigen::Matrix4f::Identity();
  reverse_initial_guess.block<3, 3>(0, 0) = initial_guess.block<3, 3>(0, 0).transpose();
  const auto reverse_coarse = runGicp(
    coarse_target, coarse_source, reverse_initial_guess,
    params_.coarse_max_correspondence_distance_m, params_.coarse_max_iterations,
    params_.transformation_epsilon, params_.euclidean_fitness_epsilon);
  DirectionResult reverse_fine;
  if (reverse_coarse.converged) {
    reverse_fine = runGicp(
      result.target_cloud, result.source_cloud, reverse_coarse.transform,
      params_.fine_max_correspondence_distance_m, params_.fine_max_iterations,
      params_.transformation_epsilon, params_.euclidean_fitness_epsilon);
  }
  result.reverse_converged = reverse_coarse.converged && reverse_fine.converged;
  result.reverse_fitness = reverse_fine.fitness;

  if (result.converged) {
    const auto overlap = calculateSymmetricOverlap(
      result.aligned_cloud, result.target_cloud, params_.overlap_max_distance_m);
    result.correspondence_count = overlap.first;
    result.overlap_ratio = overlap.second;
  }

  if (result.converged && result.reverse_converged) {
    const Eigen::Matrix4f cycle_error = reverse_fine.transform * result.final_transform;
    result.forward_reverse_translation_error = cycle_error.block<3, 1>(0, 3).norm();
    result.forward_reverse_yaw_error_deg = std::abs(normalizeYawDeg(
      static_cast<float>(std::atan2(cycle_error(1, 0), cycle_error(0, 0)) * kRadToDeg)));
  }

  result.translation_norm = result.final_transform.block<3, 1>(0, 3).norm();
  result.yaw_deg = normalizeYawDeg(static_cast<float>(
      std::atan2(result.final_transform(1, 0), result.final_transform(0, 0)) * kRadToDeg));
  return result;
}

PointCloudPtr GicpAligner::preprocess(
  const PointCloudPtr & input_cloud,
  double voxel_size_m) const
{
  PointCloudPtr no_nan(new PointCloud());
  std::vector<int> kept_indices;
  pcl::removeNaNFromPointCloud(*input_cloud, *no_nan, kept_indices);
  no_nan = removeNonFinitePoints(no_nan);

  PointCloudPtr voxelized(new PointCloud());
  pcl::VoxelGrid<PointType> voxel_filter;
  voxel_filter.setLeafSize(
    static_cast<float>(voxel_size_m),
    static_cast<float>(voxel_size_m),
    static_cast<float>(voxel_size_m));
  voxel_filter.setInputCloud(no_nan);
  voxel_filter.filter(*voxelized);
  return limitPointCount(removeNonFinitePoints(voxelized));
}

PointCloudPtr GicpAligner::limitPointCount(const PointCloudPtr & input_cloud) const
{
  if (params_.max_points == 0U || input_cloud->size() <= params_.max_points) {
    return input_cloud;
  }

  PointCloudPtr limited(new PointCloud());
  limited->reserve(params_.max_points);
  const double step = static_cast<double>(input_cloud->size()) /
    static_cast<double>(params_.max_points);
  for (std::size_t idx = 0; idx < params_.max_points; ++idx) {
    limited->push_back((*input_cloud)[static_cast<std::size_t>(idx * step)]);
  }
  limited->width = static_cast<std::uint32_t>(limited->size());
  limited->height = 1U;
  limited->is_dense = true;
  return limited;
}

float GicpAligner::normalizeYawDeg(float yaw_deg)
{
  while (yaw_deg > 180.0F) {
    yaw_deg -= 360.0F;
  }
  while (yaw_deg < -180.0F) {
    yaw_deg += 360.0F;
  }
  return yaw_deg;
}

}  // namespace pose_graph_backend
