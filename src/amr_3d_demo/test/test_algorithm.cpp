#include "amr_3d_demo/algorithm.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <string>

using amr_3d_demo::Cloud;
using amr_3d_demo::GyroSample;
using amr_3d_demo::IcpOptions;
using amr_3d_demo::Point;

TEST(Deskew, ConstantYawMakesMovingObservationsCoincide) {
  constexpr double rate = 0.9;
  const Point fixed_world(8.0, 1.0, 0.5);
  Cloud distorted;
  std::vector<double> times;
  std::vector<GyroSample> imu;
  for (int i = 0; i <= 100; ++i) imu.push_back({i * 0.01, Point(0, 0, rate)});
  for (int i = 0; i <= 10; ++i) {
    const double t = i * 0.1;
    const Eigen::Matrix3d r = Eigen::AngleAxisd(rate * t, Point::UnitZ()).toRotationMatrix();
    distorted.push_back(r.transpose() * fixed_world);
    times.push_back(t);
  }
  EXPECT_GT((distorted.front() - distorted.back()).norm(), 6.0);
  std::string reason;
  ASSERT_TRUE(amr_3d_demo::deskewRotationOnly(distorted, times, imu, 0.0, &reason)) << reason;
  for (const Point& p : distorted) EXPECT_LT((p - fixed_world).norm(), 1e-9);
}

TEST(Deskew, MissingImuCoverageLeavesCloudUntouched) {
  Cloud cloud{Point(1, 2, 3), Point(4, 5, 6)};
  const Cloud original = cloud;
  const std::vector<GyroSample> imu{{0.0, Point::Zero()}, {0.2, Point::Zero()}};
  EXPECT_FALSE(amr_3d_demo::deskewRotationOnly(cloud, {0.0, 0.5}, imu, 0.0));
  for (size_t i = 0; i < cloud.size(); ++i) EXPECT_EQ(cloud[i], original[i]);
}

TEST(GyroPrediction, IntegratesRotationBetweenScanTimes) {
  const std::vector<GyroSample> imu{{0.0, Point(0, 0, 0.6)},
                                    {0.5, Point(0, 0, 0.6)},
                                    {1.0, Point(0, 0, 0.6)}};
  Eigen::Matrix3d delta = Eigen::Matrix3d::Identity();
  ASSERT_TRUE(amr_3d_demo::integrateGyroRotation(imu, 0.25, 0.75, delta));
  EXPECT_LT((delta * Point::UnitX() - Point(std::cos(0.3), std::sin(0.3), 0)).norm(),
            1e-10);
  const Eigen::Matrix3d previous = delta;
  EXPECT_FALSE(amr_3d_demo::integrateGyroRotation(imu, 0.25, 1.2, delta));
  EXPECT_LT((delta - previous).norm(), 1e-12);
}

TEST(Icp, RecoversRigidTransformOnAsymmetricThreeDimensionalCloud) {
  Cloud map;
  for (int i = 0; i < 10; ++i)
    for (int j = 0; j < 7; ++j)
      for (int k = 0; k < 3; ++k)
        map.emplace_back(i * 0.61 + 0.03 * j, j * 0.69 + 0.02 * k,
                         k * 0.83 + 0.04 * i);
  Eigen::Isometry3d truth = Eigen::Isometry3d::Identity();
  truth.linear() = Eigen::AngleAxisd(0.055, Point(0.2, 0.3, 1.0).normalized()).toRotationMatrix();
  truth.translation() = Point(0.13, -0.08, 0.06);
  const Cloud scan = amr_3d_demo::transformCloud(map, truth.inverse());
  IcpOptions options;
  options.max_correspondence_distance = 0.6;
  const auto result = amr_3d_demo::matchPointToPoint(scan, map,
      Eigen::Isometry3d::Identity(), options);
  ASSERT_TRUE(result.success) << result.reason;
  EXPECT_GT(result.correspondences, 150);
  EXPECT_LT((result.pose.translation() - truth.translation()).norm(), 0.01);
  EXPECT_LT(Eigen::AngleAxisd(result.pose.linear() * truth.linear().transpose()).angle(), 0.01);
}

TEST(Icp, RejectsUnobservableLineGeometry) {
  Cloud line;
  for (int i = 0; i < 50; ++i) line.emplace_back(i * 0.1, 0.0, 0.0);
  const auto result = amr_3d_demo::matchPointToPoint(line, line,
      Eigen::Isometry3d::Identity());
  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.reason, "degenerate geometry");
}

TEST(Voxel, AveragesPointsWithinSameCell) {
  const Cloud filtered = amr_3d_demo::voxelDownsample(
      {Point(0.1, 0.1, 0.1), Point(0.3, 0.3, 0.3), Point(2, 0, 0)}, 1.0);
  ASSERT_EQ(filtered.size(), 2u);
  EXPECT_LT((filtered.front() - Point(0.2, 0.2, 0.2)).norm(), 1e-12);
}

TEST(Voxel, PersistentMapKeepsOldCellsAndRespectsMemoryCap) {
  amr_3d_demo::BoundedVoxelMap map(1.0, 2);
  EXPECT_TRUE(map.add({Point(0.1, 0, 0), Point(1.1, 0, 0)}));
  ASSERT_EQ(map.size(), 2u);
  EXPECT_FALSE(map.add({Point(2.1, 0, 0), Point(0.3, 0, 0)}));
  ASSERT_EQ(map.size(), 2u);
  const Cloud points = map.points();
  ASSERT_EQ(points.size(), 2u);
  EXPECT_LT((points.front() - Point(0.2, 0, 0)).norm(), 1e-12);
  EXPECT_LT((points.back() - Point(1.1, 0, 0)).norm(), 1e-12);
}
