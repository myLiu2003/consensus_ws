#include "loop_consensus/scan_context_manager.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace loop_consensus
{

namespace
{

constexpr float kNoPoint = -1000.0F;
constexpr float kTwoPi = 6.28318530717958647692F;

}  // namespace

ScanContextManager::ScanContextManager(const ScanContextParams & params)
: params_(params)
{
  unit_sector_angle_deg_ = 360.0F / static_cast<float>(params_.num_sectors);
}

Eigen::MatrixXf ScanContextManager::makeScanContext(const PointCloud & scan) const
{
  Eigen::MatrixXf desc =
    Eigen::MatrixXf::Constant(params_.num_rings, params_.num_sectors, kNoPoint);

  for (const auto & point : scan.points) {
    const float range_xy = std::sqrt(point.x * point.x + point.y * point.y);
    if (range_xy <= 0.0F || range_xy > static_cast<float>(params_.max_radius_m)) {
      continue;
    }

    const float angle_deg = xyToThetaDeg(point.x, point.y);
    const int ring_idx = std::clamp(
      static_cast<int>(std::ceil((range_xy / static_cast<float>(params_.max_radius_m)) *
      static_cast<float>(params_.num_rings))) - 1,
      0,
      params_.num_rings - 1);
    const int sector_idx = std::clamp(
      static_cast<int>(std::ceil((angle_deg / 360.0F) * static_cast<float>(params_.num_sectors))) - 1,
      0,
      params_.num_sectors - 1);

    const float value = point.z + static_cast<float>(params_.lidar_height_m);
    desc(ring_idx, sector_idx) = std::max(desc(ring_idx, sector_idx), value);
  }

  for (int row = 0; row < desc.rows(); ++row) {
    for (int col = 0; col < desc.cols(); ++col) {
      if (desc(row, col) == kNoPoint) {
        desc(row, col) = 0.0F;
      }
    }
  }

  return desc;
}

Eigen::VectorXf ScanContextManager::makeRingKey(const Eigen::MatrixXf & desc) const
{
  Eigen::VectorXf ring_key(desc.rows());
  for (int row = 0; row < desc.rows(); ++row) {
    ring_key(row) = desc.row(row).mean();
  }
  return ring_key;
}

Eigen::RowVectorXf ScanContextManager::makeSectorKey(const Eigen::MatrixXf & desc) const
{
  Eigen::RowVectorXf sector_key(desc.cols());
  for (int col = 0; col < desc.cols(); ++col) {
    sector_key(col) = desc.col(col).mean();
  }
  return sector_key;
}

int ScanContextManager::fastAlignUsingSectorKey(
  const Eigen::RowVectorXf & key1,
  const Eigen::RowVectorXf & key2) const
{
  int best_shift = 0;
  float best_norm = std::numeric_limits<float>::max();
  for (int shift = 0; shift < key1.cols(); ++shift) {
    const Eigen::RowVectorXf shifted = circularShift(key2, shift);
    const float diff_norm = (key1 - shifted).norm();
    if (diff_norm < best_norm) {
      best_norm = diff_norm;
      best_shift = shift;
    }
  }
  return best_shift;
}

float ScanContextManager::directDistance(
  const Eigen::MatrixXf & sc1,
  const Eigen::MatrixXf & sc2) const
{
  int effective_cols = 0;
  float sum_similarity = 0.0F;

  for (int col = 0; col < sc1.cols(); ++col) {
    const Eigen::VectorXf col1 = sc1.col(col);
    const Eigen::VectorXf col2 = sc2.col(col);
    const float norm1 = col1.norm();
    const float norm2 = col2.norm();
    if (norm1 == 0.0F || norm2 == 0.0F) {
      continue;
    }

    const float similarity = col1.dot(col2) / (norm1 * norm2);
    sum_similarity += similarity;
    ++effective_cols;
  }

  if (effective_cols == 0) {
    return 1.0F;
  }

  return 1.0F - (sum_similarity / static_cast<float>(effective_cols));
}

std::pair<float, int> ScanContextManager::distanceBetweenScanContexts(
  const Eigen::MatrixXf & sc1,
  const Eigen::MatrixXf & sc2,
  const Eigen::RowVectorXf & sector_key1,
  const Eigen::RowVectorXf & sector_key2) const
{
  const int coarse_shift = fastAlignUsingSectorKey(sector_key1, sector_key2);
  const int search_radius = std::max(
    1,
    static_cast<int>(std::round(0.5 * params_.search_ratio * static_cast<double>(sc1.cols()))));

  std::vector<int> search_space;
  search_space.reserve(static_cast<std::size_t>(2 * search_radius + 1));
  search_space.push_back(coarse_shift);
  for (int offset = 1; offset <= search_radius; ++offset) {
    search_space.push_back((coarse_shift + offset + sc1.cols()) % sc1.cols());
    search_space.push_back((coarse_shift - offset + sc1.cols()) % sc1.cols());
  }
  std::sort(search_space.begin(), search_space.end());
  search_space.erase(std::unique(search_space.begin(), search_space.end()), search_space.end());

  int best_shift = 0;
  float best_distance = std::numeric_limits<float>::max();
  for (const int shift : search_space) {
    const Eigen::MatrixXf shifted = circularShift(sc2, shift);
    const float dist = directDistance(sc1, shifted);
    if (dist < best_distance) {
      best_distance = dist;
      best_shift = shift;
    }
  }

  return {best_distance, best_shift};
}

float ScanContextManager::shiftToYawRad(int shift) const
{
  return static_cast<float>(shift) * unit_sector_angle_deg_ * static_cast<float>(M_PI / 180.0);
}

float ScanContextManager::xyToThetaDeg(float x, float y)
{
  float angle = std::atan2(y, x) * 180.0F / static_cast<float>(M_PI);
  if (angle < 0.0F) {
    angle += 360.0F;
  }
  return angle;
}

Eigen::MatrixXf ScanContextManager::circularShift(const Eigen::MatrixXf & mat, int shift)
{
  if (shift == 0) {
    return mat;
  }

  Eigen::MatrixXf shifted = Eigen::MatrixXf::Zero(mat.rows(), mat.cols());
  for (int col = 0; col < mat.cols(); ++col) {
    const int new_location = (col + shift) % mat.cols();
    shifted.col(new_location) = mat.col(col);
  }
  return shifted;
}

Eigen::RowVectorXf ScanContextManager::circularShift(const Eigen::RowVectorXf & row, int shift)
{
  if (shift == 0) {
    return row;
  }

  Eigen::RowVectorXf shifted = Eigen::RowVectorXf::Zero(row.cols());
  for (int col = 0; col < row.cols(); ++col) {
    const int new_location = (col + shift) % row.cols();
    shifted(new_location) = row(col);
  }
  return shifted;
}

}  // namespace loop_consensus
