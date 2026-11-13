#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <string>
#include <map>
#include <tuple>
#include <vector>

namespace amr_3d_demo {

using Point = Eigen::Vector3d;
using Cloud = std::vector<Point>;

struct GyroSample {
  double time_sec;
  Point angular_velocity_lidar;
};

// All point times are absolute seconds on the same clock as the IMU samples.
// Re-express each LiDAR point in the LiDAR frame at reference_time_sec.
// Only angular motion is modeled; translation and the IMU-LiDAR lever arm are ignored.
// On failure the input cloud is left untouched.
bool deskewRotationOnly(Cloud& cloud, const std::vector<double>& point_times_sec,
                        const std::vector<GyroSample>& imu, double reference_time_sec,
                        std::string* reason = nullptr);

// Relative orientation from LiDAR frame at from_sec to LiDAR frame at to_sec,
// integrated from gyro samples already rotated into LiDAR axes.
// On failure the caller's output matrix is unchanged.
bool integrateGyroRotation(const std::vector<GyroSample>& imu, double from_sec,
                           double to_sec, Eigen::Matrix3d& relative_rotation,
                           std::string* reason = nullptr);

struct IcpOptions {
  int max_iterations = 10;
  int min_correspondences = 30;
  double max_correspondence_distance = 0.8;
  double huber_delta = 0.20;
};

struct IcpResult {
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  int correspondences = 0;
  int iterations = 0;
  double rmse = 0.0;
  bool success = false;
  std::string reason;
};

// Scan points are in LiDAR frame; target points are in the demo map frame.
// Minimize sum_i rho(|| T * p_i - nearest_target(T * p_i) ||^2).
// Correspondence search uses a KD-tree; the SE(3) Gauss-Newton update is ours.
IcpResult matchPointToPoint(const Cloud& scan, const Cloud& target,
                            const Eigen::Isometry3d& initial_pose,
                            const IcpOptions& options = {});

Cloud voxelDownsample(const Cloud& cloud, double leaf_size);
Cloud transformCloud(const Cloud& cloud, const Eigen::Isometry3d& pose);

// Persistent, bounded voxel map for visualization. Unlike the local ICP map,
// old keyframes never leave it. Existing cells can still update at capacity.
class BoundedVoxelMap {
 public:
  BoundedVoxelMap(double leaf_size, size_t max_cells);
  bool add(const Cloud& cloud);  // false if some new cells were refused at capacity
  Cloud points() const;
  size_t size() const { return cells_.size(); }

 private:
  struct Cell { Point sum = Point::Zero(); size_t count = 0; };
  double leaf_size_;
  size_t max_cells_;
  std::map<std::tuple<int, int, int>, Cell> cells_;
};

}  // namespace amr_3d_demo
