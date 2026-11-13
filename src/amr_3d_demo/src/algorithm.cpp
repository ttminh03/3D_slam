#include "amr_3d_demo/algorithm.hpp"

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Eigenvalues>
#include <Eigen/Cholesky>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <stdexcept>
#include <tuple>

namespace amr_3d_demo {
namespace {

Eigen::Matrix3d skew(const Point& p) {
  Eigen::Matrix3d s;
  s << 0, -p.z(), p.y(), p.z(), 0, -p.x(), -p.y(), p.x(), 0;
  return s;
}

Eigen::Matrix3d expSO3(const Point& omega) {
  const double angle = omega.norm();
  if (angle < 1e-12) return Eigen::Matrix3d::Identity() + skew(omega);
  return Eigen::AngleAxisd(angle, omega / angle).toRotationMatrix();
}

bool sampleRotation(const std::vector<GyroSample>& imu,
                    const std::vector<Eigen::Matrix3d>& rotations,
                    double query, Eigen::Matrix3d& out) {
  if (query < imu.front().time_sec - 1e-8 || query > imu.back().time_sec + 1e-8)
    return false;
  if (query <= imu.front().time_sec) { out = rotations.front(); return true; }
  if (query >= imu.back().time_sec) { out = rotations.back(); return true; }
  auto upper = std::upper_bound(imu.begin(), imu.end(), query,
      [](double t, const GyroSample& s) { return t < s.time_sec; });
  const size_t left = static_cast<size_t>(upper - imu.begin() - 1);
  const double dt = query - imu[left].time_sec;
  const double full_dt = imu[left + 1].time_sec - imu[left].time_sec;
  const Point omega_query = imu[left].angular_velocity_lidar +
      (imu[left + 1].angular_velocity_lidar - imu[left].angular_velocity_lidar) * (dt / full_dt);
  out = rotations[left] * expSO3(0.5 * (imu[left].angular_velocity_lidar + omega_query) * dt);
  return true;
}

}  // namespace

bool deskewRotationOnly(Cloud& cloud, const std::vector<double>& times,
                        const std::vector<GyroSample>& imu, double reference,
                        std::string* reason) {
  auto fail = [&](const char* message) {
    if (reason) *reason = message;
    return false;
  };
  if (cloud.size() != times.size() || cloud.empty()) return fail("point time count mismatch");
  if (imu.size() < 2) return fail("fewer than two IMU samples");
  if (!std::isfinite(reference)) return fail("invalid reference time");
  for (size_t i = 0; i < imu.size(); ++i) {
    if (!std::isfinite(imu[i].time_sec) || !imu[i].angular_velocity_lidar.allFinite())
      return fail("invalid IMU sample");
    if (i && imu[i].time_sec <= imu[i - 1].time_sec)
      return fail("IMU samples are not strictly ordered");
  }
  if (reference < imu.front().time_sec - 1e-8 || reference > imu.back().time_sec + 1e-8)
    return fail("IMU does not cover reference time");
  for (double t : times)
    if (!std::isfinite(t) || t < imu.front().time_sec - 1e-8 ||
        t > imu.back().time_sec + 1e-8)
      return fail("IMU does not cover all point times");

  std::vector<Eigen::Matrix3d> rotations(imu.size(), Eigen::Matrix3d::Identity());
  for (size_t i = 1; i < imu.size(); ++i) {
    const double dt = imu[i].time_sec - imu[i - 1].time_sec;
    const Point mean_omega = 0.5 * (imu[i - 1].angular_velocity_lidar +
                                     imu[i].angular_velocity_lidar);
    rotations[i] = rotations[i - 1] * expSO3(mean_omega * dt);
  }
  Eigen::Matrix3d reference_rotation;
  sampleRotation(imu, rotations, reference, reference_rotation);
  Cloud corrected;
  corrected.reserve(cloud.size());
  for (size_t i = 0; i < cloud.size(); ++i) {
    Eigen::Matrix3d point_rotation;
    sampleRotation(imu, rotations, times[i], point_rotation);
    corrected.push_back(reference_rotation.transpose() * point_rotation * cloud[i]);
  }
  cloud.swap(corrected);
  if (reason) *reason = "rotation-only deskew applied";
  return true;
}

bool integrateGyroRotation(const std::vector<GyroSample>& imu, double from,
                           double to, Eigen::Matrix3d& relative_rotation,
                           std::string* reason) {
  auto fail = [&](const char* message) {
    if (reason) *reason = message;
    return false;
  };
  if (imu.size() < 2) return fail("fewer than two IMU samples");
  if (!std::isfinite(from) || !std::isfinite(to) || to < from)
    return fail("invalid integration interval");
  if (from < imu.front().time_sec - 1e-8 || to > imu.back().time_sec + 1e-8)
    return fail("IMU does not cover integration interval");
  std::vector<Eigen::Matrix3d> rotations(imu.size(), Eigen::Matrix3d::Identity());
  for (size_t i = 0; i < imu.size(); ++i) {
    if (!std::isfinite(imu[i].time_sec) || !imu[i].angular_velocity_lidar.allFinite())
      return fail("invalid IMU sample");
    if (i == 0) continue;
    const double dt = imu[i].time_sec - imu[i - 1].time_sec;
    if (dt <= 0) return fail("IMU samples are not strictly ordered");
    rotations[i] = rotations[i - 1] * expSO3(
        0.5 * (imu[i - 1].angular_velocity_lidar + imu[i].angular_velocity_lidar) * dt);
  }
  Eigen::Matrix3d start_rotation, end_rotation;
  if (!sampleRotation(imu, rotations, from, start_rotation) ||
      !sampleRotation(imu, rotations, to, end_rotation))
    return fail("IMU interpolation failed");
  relative_rotation = start_rotation.transpose() * end_rotation;
  if (reason) *reason = "gyro rotation integrated";
  return true;
}

Cloud voxelDownsample(const Cloud& cloud, double leaf) {
  if (!(leaf > 0.0) || !std::isfinite(leaf)) return {};
  using Key = std::tuple<int, int, int>;
  struct Cell { Point sum = Point::Zero(); int count = 0; };
  std::map<Key, Cell> cells;
  for (const Point& p : cloud) {
    if (!p.allFinite()) continue;
    const Key key{static_cast<int>(std::floor(p.x() / leaf)),
                  static_cast<int>(std::floor(p.y() / leaf)),
                  static_cast<int>(std::floor(p.z() / leaf))};
    auto& cell = cells[key];
    cell.sum += p;
    ++cell.count;
  }
  Cloud result;
  result.reserve(cells.size());
  for (const auto& [key, cell] : cells) result.push_back(cell.sum / cell.count);
  return result;
}

Cloud transformCloud(const Cloud& cloud, const Eigen::Isometry3d& pose) {
  Cloud result;
  result.reserve(cloud.size());
  for (const Point& p : cloud) result.push_back(pose * p);
  return result;
}

BoundedVoxelMap::BoundedVoxelMap(double leaf_size, size_t max_cells)
    : leaf_size_(leaf_size), max_cells_(max_cells) {
  if (!(leaf_size > 0) || !std::isfinite(leaf_size) || max_cells == 0)
    throw std::invalid_argument("voxel size and maximum cell count must be positive");
}

bool BoundedVoxelMap::add(const Cloud& cloud) {
  bool complete = true;
  for (const Point& p : cloud) {
    if (!p.allFinite()) continue;
    const std::tuple<int, int, int> key{
        static_cast<int>(std::floor(p.x() / leaf_size_)),
        static_cast<int>(std::floor(p.y() / leaf_size_)),
        static_cast<int>(std::floor(p.z() / leaf_size_))};
    auto it = cells_.find(key);
    if (it == cells_.end()) {
      if (cells_.size() >= max_cells_) { complete = false; continue; }
      it = cells_.emplace(key, Cell{}).first;
    }
    it->second.sum += p;
    ++it->second.count;
  }
  return complete;
}

Cloud BoundedVoxelMap::points() const {
  Cloud result;
  result.reserve(cells_.size());
  for (const auto& [key, cell] : cells_)
    result.push_back(cell.sum / static_cast<double>(cell.count));
  return result;
}

IcpResult matchPointToPoint(const Cloud& scan, const Cloud& target,
                            const Eigen::Isometry3d& initial_pose,
                            const IcpOptions& options) {
  IcpResult result;
  result.pose = initial_pose;
  if (scan.size() < static_cast<size_t>(options.min_correspondences) ||
      target.size() < static_cast<size_t>(options.min_correspondences)) {
    result.reason = "too few input points";
    return result;
  }
  if (options.max_iterations < 1 || options.min_correspondences < 6 ||
      options.max_correspondence_distance <= 0 || options.huber_delta <= 0) {
    result.reason = "invalid ICP options";
    return result;
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_pcl(new pcl::PointCloud<pcl::PointXYZ>);
  target_pcl->reserve(target.size());
  for (const Point& p : target) {
    if (!p.allFinite()) { result.reason = "nonfinite map point"; return result; }
    target_pcl->emplace_back(static_cast<float>(p.x()), static_cast<float>(p.y()),
                             static_cast<float>(p.z()));
  }
  pcl::KdTreeFLANN<pcl::PointXYZ> tree;
  tree.setInputCloud(target_pcl);
  const double max_d2 = options.max_correspondence_distance *
                        options.max_correspondence_distance;
  for (int iteration = 0; iteration < options.max_iterations; ++iteration) {
    Eigen::Matrix<double, 6, 6> h = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 1> g = Eigen::Matrix<double, 6, 1>::Zero();
    double sum_sq = 0.0;
    int count = 0;
    for (const Point& p : scan) {
      if (!p.allFinite()) continue;
      const Point transformed = result.pose * p;
      const pcl::PointXYZ query(static_cast<float>(transformed.x()),
                                static_cast<float>(transformed.y()),
                                static_cast<float>(transformed.z()));
      std::vector<int> indices(1);
      std::vector<float> distances(1);
      if (tree.nearestKSearch(query, 1, indices, distances) != 1 ||
          distances[0] > max_d2) continue;
      const Point residual = transformed - target[indices[0]];
      const double norm = residual.norm();
      const double weight = norm > options.huber_delta ? options.huber_delta / norm : 1.0;
      // Left update exp(delta) T: d(Tp)/d(delta) = [-[Tp]_x, I].
      Eigen::Matrix<double, 3, 6> jacobian;
      jacobian.leftCols<3>() = -skew(transformed);
      jacobian.rightCols<3>().setIdentity();
      h.noalias() += weight * jacobian.transpose() * jacobian;
      g.noalias() += weight * jacobian.transpose() * residual;
      sum_sq += residual.squaredNorm();
      ++count;
    }
    result.iterations = iteration + 1;
    result.correspondences = count;
    result.rmse = count ? std::sqrt(sum_sq / count) : 0.0;
    if (count < options.min_correspondences) {
      result.reason = "too few correspondences";
      return result;
    }
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> spectrum(h);
    if (spectrum.info() != Eigen::Success ||
        spectrum.eigenvalues().minCoeff() < 1e-7 * spectrum.eigenvalues().maxCoeff()) {
      result.reason = "degenerate geometry";
      return result;
    }
    const Eigen::Matrix<double, 6, 1> delta = -h.ldlt().solve(g);
    if (!delta.allFinite() || delta.head<3>().norm() > 0.35 ||
        delta.tail<3>().norm() > 0.7) {
      result.reason = "unstable ICP step";
      return result;
    }
    const Eigen::Matrix3d dr = expSO3(delta.head<3>());
    result.pose.translation() = dr * result.pose.translation() + delta.tail<3>();
    result.pose.linear() = dr * result.pose.linear();
    if (delta.head<3>().norm() < 1e-4 && delta.tail<3>().norm() < 1e-3) break;
  }
  result.success = true;
  result.reason = "matched";
  return result;
}

}  // namespace amr_3d_demo
