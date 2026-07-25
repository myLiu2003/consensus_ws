#pragma once

#include "pose_graph_backend/types.hpp"

namespace pose_graph_backend
{

class GicpAligner
{
public:
  explicit GicpAligner(const GicpParameters & params);

  GicpResult align(
    const SubmapData & source_submap,
    const SubmapData & target_submap,
    float yaw_initial_rad) const;

private:
  PointCloudPtr preprocess(const PointCloudPtr & input_cloud, double voxel_size_m) const;
  PointCloudPtr limitPointCount(const PointCloudPtr & input_cloud) const;
  static float normalizeYawDeg(float yaw_deg);

  GicpParameters params_;
};

}  // namespace pose_graph_backend
