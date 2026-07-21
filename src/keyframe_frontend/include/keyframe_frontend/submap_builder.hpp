#pragma once

#include <deque>

#include "keyframe_frontend/keyframe_types.hpp"

namespace keyframe_frontend
{

class SubmapBuilder
{
public:
  explicit SubmapBuilder(SubmapParams params);

  PointCloudPtr preprocessSingleFrame(const PointCloudConstPtr & input_cloud) const;

  PointCloudPtr buildSubmap(
    const std::deque<KeyframeData> & keyframes,
    std::size_t center_index) const;

private:
  PointCloudPtr cropByRange(const PointCloudConstPtr & input_cloud) const;
  PointCloudPtr removeBodyNearbyPoints(const PointCloudConstPtr & input_cloud) const;
  PointCloudPtr voxelDownsample(const PointCloudConstPtr & input_cloud) const;
  PointCloudPtr limitPointCount(const PointCloudConstPtr & input_cloud) const;

  SubmapParams params_;
};

}  // namespace keyframe_frontend
