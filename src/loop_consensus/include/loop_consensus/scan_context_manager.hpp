#pragma once

#include <utility>
#include <vector>

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace loop_consensus
{

struct ScanContextParams
{
  int num_rings{20};
  int num_sectors{60};
  double max_radius_m{40.0};
  double lidar_height_m{2.0};
  double search_ratio{0.1};
};

class ScanContextManager
{
public:
  using PointType = pcl::PointXYZ;
  using PointCloud = pcl::PointCloud<PointType>;

  explicit ScanContextManager(const ScanContextParams & params);

  Eigen::MatrixXf makeScanContext(const PointCloud & scan) const;
  Eigen::VectorXf makeRingKey(const Eigen::MatrixXf & desc) const;
  Eigen::RowVectorXf makeSectorKey(const Eigen::MatrixXf & desc) const;

  int fastAlignUsingSectorKey(
    const Eigen::RowVectorXf & key1,
    const Eigen::RowVectorXf & key2) const;

  float directDistance(
    const Eigen::MatrixXf & sc1,
    const Eigen::MatrixXf & sc2) const;

  std::pair<float, int> distanceBetweenScanContexts(
    const Eigen::MatrixXf & sc1,
    const Eigen::MatrixXf & sc2,
    const Eigen::RowVectorXf & sector_key1,
    const Eigen::RowVectorXf & sector_key2) const;

  float shiftToYawRad(int shift) const;

private:
  static float xyToThetaDeg(float x, float y);
  static Eigen::MatrixXf circularShift(const Eigen::MatrixXf & mat, int shift);
  static Eigen::RowVectorXf circularShift(const Eigen::RowVectorXf & row, int shift);

  ScanContextParams params_;
  float unit_sector_angle_deg_{6.0F};
};

}  // namespace loop_consensus
