#include "keyframe_frontend/submap_builder.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>

namespace keyframe_frontend
{

SubmapBuilder::SubmapBuilder(SubmapParams params)
: params_(std::move(params))
{
}

PointCloudPtr SubmapBuilder::preprocessSingleFrame(const PointCloudConstPtr & input_cloud) const
{
  PointCloudPtr no_nan_cloud(new PointCloud());
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*input_cloud, *no_nan_cloud, indices);

  auto cropped_cloud = cropByRange(no_nan_cloud);
  auto trimmed_cloud = removeBodyNearbyPoints(cropped_cloud);
  auto downsampled_cloud = voxelDownsample(trimmed_cloud);
  return limitPointCount(downsampled_cloud);
}

PointCloudPtr SubmapBuilder::buildSubmap(
  const std::deque<KeyframeData> & keyframes,
  const std::size_t center_index) const
{
  if (keyframes.empty() || center_index >= keyframes.size()) {
    return PointCloudPtr(new PointCloud());
  }

  const auto & center = keyframes[center_index];
  const int begin_index =
    std::max(0, static_cast<int>(center_index) - params_.window_size);
  const int end_index =
    std::min(static_cast<int>(keyframes.size()) - 1, static_cast<int>(center_index) + params_.window_size);

  PointCloudPtr merged_cloud(new PointCloud());
  for (int idx = begin_index; idx <= end_index; ++idx) {
    const auto & source = keyframes[static_cast<std::size_t>(idx)];
    if (!source.local_cloud || source.local_cloud->empty()) {
      continue;
    }

    // 所有单帧点云都保存在各自关键帧局部系，这里通过相对位姿统一到中心关键帧系。
    const Eigen::Isometry3d relative_transform =
      center.pose_odom_to_keyframe.inverse() * source.pose_odom_to_keyframe;
    PointCloud transformed_cloud;
    pcl::transformPointCloud(
      *source.local_cloud,
      transformed_cloud,
      relative_transform.matrix().cast<float>());
    *merged_cloud += transformed_cloud;
  }

  PointCloudPtr merged_ptr(new PointCloud(*merged_cloud));
  auto cropped_cloud = cropByRange(merged_ptr);
  auto trimmed_cloud = removeBodyNearbyPoints(cropped_cloud);
  auto downsampled_cloud = voxelDownsample(trimmed_cloud);
  return limitPointCount(downsampled_cloud);
}

PointCloudPtr SubmapBuilder::cropByRange(const PointCloudConstPtr & input_cloud) const
{
  PointCloudPtr cropped_cloud(new PointCloud());
  cropped_cloud->reserve(input_cloud->size());

  const double min_sq = params_.range_min_m * params_.range_min_m;
  const double max_sq = params_.range_max_m * params_.range_max_m;
  for (const auto & point : input_cloud->points) {
    const double dist_sq =
      static_cast<double>(point.x) * point.x +
      static_cast<double>(point.y) * point.y +
      static_cast<double>(point.z) * point.z;
    if (dist_sq < min_sq || dist_sq > max_sq) {
      continue;
    }
    cropped_cloud->push_back(point);
  }
  cropped_cloud->width = static_cast<uint32_t>(cropped_cloud->size());
  cropped_cloud->height = 1U;
  cropped_cloud->is_dense = true;
  return cropped_cloud;
}

PointCloudPtr SubmapBuilder::removeBodyNearbyPoints(const PointCloudConstPtr & input_cloud) const
{
  PointCloudPtr trimmed_cloud(new PointCloud());
  trimmed_cloud->reserve(input_cloud->size());

  const double body_sq = params_.body_exclusion_radius_m * params_.body_exclusion_radius_m;
  for (const auto & point : input_cloud->points) {
    const double dist_sq =
      static_cast<double>(point.x) * point.x +
      static_cast<double>(point.y) * point.y +
      static_cast<double>(point.z) * point.z;
    if (dist_sq < body_sq) {
      continue;
    }
    trimmed_cloud->push_back(point);
  }
  trimmed_cloud->width = static_cast<uint32_t>(trimmed_cloud->size());
  trimmed_cloud->height = 1U;
  trimmed_cloud->is_dense = true;
  return trimmed_cloud;
}

PointCloudPtr SubmapBuilder::voxelDownsample(const PointCloudConstPtr & input_cloud) const
{
  if (params_.voxel_size_m <= 0.0) {
    return PointCloudPtr(new PointCloud(*input_cloud));
  }

  pcl::VoxelGrid<PointType> voxel_filter;
  voxel_filter.setInputCloud(input_cloud);
  voxel_filter.setLeafSize(
    static_cast<float>(params_.voxel_size_m),
    static_cast<float>(params_.voxel_size_m),
    static_cast<float>(params_.voxel_size_m));

  PointCloudPtr downsampled_cloud(new PointCloud());
  voxel_filter.filter(*downsampled_cloud);
  return downsampled_cloud;
}

PointCloudPtr SubmapBuilder::limitPointCount(const PointCloudConstPtr & input_cloud) const
{
  if (params_.max_points == 0U || input_cloud->size() <= params_.max_points) {
    return PointCloudPtr(new PointCloud(*input_cloud));
  }

  PointCloudPtr limited_cloud(new PointCloud());
  limited_cloud->reserve(params_.max_points);

  const double step = static_cast<double>(input_cloud->size()) / static_cast<double>(params_.max_points);
  for (std::size_t idx = 0; idx < params_.max_points; ++idx) {
    const std::size_t src_index = std::min(
      static_cast<std::size_t>(std::floor(idx * step)),
      input_cloud->size() - 1U);
    limited_cloud->push_back(input_cloud->points[src_index]);
  }

  limited_cloud->width = static_cast<uint32_t>(limited_cloud->size());
  limited_cloud->height = 1U;
  limited_cloud->is_dense = true;
  return limited_cloud;
}

}  // namespace keyframe_frontend
