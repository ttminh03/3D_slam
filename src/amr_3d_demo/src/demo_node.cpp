#include "amr_3d_demo/algorithm.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <deque>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace amr_3d_demo {
namespace {

// Keep the estimated pose in its own TF branch. `odom` belongs to Gazebo's
// odom -> base_footprint tree and must never be this demo's TF child.
constexpr std::string_view kEstimatedPoseFrame = "demo_lidar_pose";
static_assert(kEstimatedPoseFrame != "odom" && kEstimatedPoseFrame != "lidar_link" &&
              kEstimatedPoseFrame != "base_link" &&
              kEstimatedPoseFrame != "base_footprint",
              "Estimated pose frame must not reuse a robot/Gazebo TF child");

const sensor_msgs::msg::PointField* findField(const sensor_msgs::msg::PointCloud2& msg,
                                               const std::string& name) {
  for (const auto& field : msg.fields) if (field.name == name) return &field;
  return nullptr;
}

double readNumber(const uint8_t* data, const sensor_msgs::msg::PointField& field) {
  if (field.datatype == sensor_msgs::msg::PointField::FLOAT32) {
    float value;
    std::memcpy(&value, data + field.offset, sizeof(value));
    return value;
  }
  double value;
  std::memcpy(&value, data + field.offset, sizeof(value));
  return value;
}

bool validField(const sensor_msgs::msg::PointCloud2& msg,
                const sensor_msgs::msg::PointField* field) {
  if (!field || field->count != 1) return false;
  const unsigned bytes = field->datatype == sensor_msgs::msg::PointField::FLOAT32 ? 4 :
                         field->datatype == sensor_msgs::msg::PointField::FLOAT64 ? 8 : 0;
  return bytes && field->offset + bytes <= msg.point_step;
}

geometry_msgs::msg::Pose poseMsg(const Eigen::Isometry3d& pose) {
  geometry_msgs::msg::Pose out;
  out.position.x = pose.translation().x();
  out.position.y = pose.translation().y();
  out.position.z = pose.translation().z();
  const Eigen::Quaterniond q(pose.linear());
  out.orientation.x = q.x();
  out.orientation.y = q.y();
  out.orientation.z = q.z();
  out.orientation.w = q.w();
  return out;
}

sensor_msgs::msg::PointCloud2 cloudMsg(const Cloud& cloud, const std_msgs::msg::Header& header) {
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  pcl_cloud.reserve(cloud.size());
  for (const Point& p : cloud)
    pcl_cloud.emplace_back(static_cast<float>(p.x()), static_cast<float>(p.y()),
                           static_cast<float>(p.z()));
  sensor_msgs::msg::PointCloud2 out;
  pcl::toROSMsg(pcl_cloud, out);
  out.header = header;
  return out;
}

}  // namespace

class DemoNode final : public rclcpp::Node {
 public:
  DemoNode() : Node("amr_3d_demo"),
               tf_buffer_(std::make_unique<tf2_ros::Buffer>(get_clock())),
               tf_listener_(std::make_unique<tf2_ros::TransformListener>(*tf_buffer_)) {
    scan_leaf_ = declare_parameter<double>("scan_voxel_size", 0.25);
    map_leaf_ = declare_parameter<double>("map_voxel_size", 0.35);
    global_leaf_ = declare_parameter<double>("global_voxel_size", 0.35);
    max_scan_points_ = declare_parameter<int>("max_scan_points", 500);
    max_global_voxels_ = declare_parameter<int>("max_global_voxels", 30000);
    max_range_ = declare_parameter<double>("max_range", 35.0);
    keyframe_distance_ = declare_parameter<double>("keyframe_distance", 0.30);
    keyframe_angle_ = declare_parameter<double>("keyframe_angle", 0.20);
    max_keyframes_ = declare_parameter<int>("max_local_keyframes", 8);
    const std::string point_topic = declare_parameter<std::string>("point_topic", "/points");
    const std::string imu_topic = declare_parameter<std::string>("imu_topic", "/imu/data");
    if (scan_leaf_ <= 0 || map_leaf_ <= 0 || global_leaf_ <= 0 || max_range_ <= 0 ||
        keyframe_distance_ <= 0 || keyframe_angle_ <= 0 || max_keyframes_ < 1 ||
        max_scan_points_ < 30 || max_global_voxels_ < 1)
      throw std::invalid_argument("all voxel/range/keyframe parameters must be positive");
    global_map_ = std::make_unique<BoundedVoxelMap>(global_leaf_, max_global_voxels_);

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/demo/odom", 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/demo/path", 10);
    map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/demo/local_map", 1);
    global_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/demo/map", rclcpp::QoS(1).transient_local());
    status_pub_ = create_publisher<std_msgs::msg::String>("/demo/status", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    points_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        point_topic, rclcpp::SensorDataQoS().keep_last(1),
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { onPoints(msg); });
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, rclcpp::SensorDataQoS().keep_last(512),
        [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) { onImu(*msg); });
    pending_timer_ = create_wall_timer(std::chrono::milliseconds(20),
                                      [this]() { processReadyScans(); });
    global_map_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
      if (initialized_) publishGlobalMap(now());
    });
    path_.header.frame_id = "demo_map";
    RCLCPP_INFO(get_logger(), "3D demo: points=%s imu=%s; LiDAR pose is published as demo_map -> demo_lidar_pose",
                point_topic.c_str(), imu_topic.c_str());
  }

 private:
  void status(const std::string& value) {
    std_msgs::msg::String msg;
    msg.data = value;
    status_pub_->publish(msg);
  }

  void onImu(const sensor_msgs::msg::Imu& msg) {
    if (lidar_frame_.empty() || msg.header.frame_id.empty()) return;
    if (imu_frame_ != msg.header.frame_id || !imu_to_lidar_) {
      try {
        const auto tf = tf_buffer_->lookupTransform(lidar_frame_, msg.header.frame_id,
                                                    tf2::TimePointZero);
        const auto& q = tf.transform.rotation;
        imu_to_lidar_ = Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized().toRotationMatrix();
        imu_frame_ = msg.header.frame_id;
        imu_.clear();
      } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                             "Waiting for IMU->LiDAR TF: %s", ex.what());
        return;
      }
    }
    const Point omega(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);
    const double stamp = rclcpp::Time(msg.header.stamp).seconds();
    if (!omega.allFinite() || !std::isfinite(stamp)) return;
    if (!imu_.empty() && stamp <= imu_.back().time_sec) {
      // A new /clock epoch or out-of-order IMU data must not be integrated.
      if (stamp < imu_.back().time_sec - 0.5) imu_.clear();
      else return;
    }
    imu_.push_back({stamp, *imu_to_lidar_ * omega});
    while (imu_.size() > 2 && stamp - imu_.front().time_sec > 3.0) imu_.pop_front();
    processReadyScans();
  }

  struct PendingScan {
    sensor_msgs::msg::PointCloud2::ConstSharedPtr message;
    std::chrono::steady_clock::time_point arrival;
    double needed_imu_time;
  };

  void onPoints(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
    if (msg->header.frame_id.empty() ||
        (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0)) {
      status("rejected: missing frame or timestamp");
      return;
    }
    if (lidar_frame_.empty()) lidar_frame_ = msg->header.frame_id;
    if (msg->header.frame_id != lidar_frame_) {
      status("rejected: LiDAR frame changed");
      return;
    }
    double needed_imu_time = rclcpp::Time(msg->header.stamp).seconds();
    const auto* t = findField(*msg, "time");
    if (t && validField(*msg, t) && !msg->is_bigendian &&
        msg->point_step && msg->row_step >= msg->width * msg->point_step &&
        msg->data.size() >= static_cast<size_t>(msg->height) * msg->row_step) {
      // A point-time cloud may require IMU samples through its final point.
      for (uint32_t row = 0; row < msg->height; ++row) {
        for (uint32_t col = 0; col < msg->width; ++col) {
          const uint8_t* ptr = msg->data.data() + row * msg->row_step + col * msg->point_step;
          const double relative = readNumber(ptr, *t);
          if (std::isfinite(relative) && relative >= 0 && relative <= 2.0)
            needed_imu_time = std::max(needed_imu_time,
                                       rclcpp::Time(msg->header.stamp).seconds() + relative);
        }
      }
    }
    pending_scans_.push_back({msg, std::chrono::steady_clock::now(), needed_imu_time});
    while (pending_scans_.size() > 5) {
      pending_scans_.pop_front();
      status("dropped: IMU wait queue overflow");
    }
    processReadyScans();
  }

  void processReadyScans() {
    while (!pending_scans_.empty()) {
      const auto& pending = pending_scans_.front();
      const double stamp = rclcpp::Time(pending.message->header.stamp).seconds();
      const bool covered = imu_.size() >= 2 && imu_.front().time_sec <= stamp &&
                           imu_.back().time_sec >= pending.needed_imu_time;
      const bool expired = std::chrono::steady_clock::now() - pending.arrival >=
                           std::chrono::milliseconds(180);
      if (!covered && !expired) return;
      const auto msg = pending.message;
      const double wait_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - pending.arrival).count();
      pending_scans_.pop_front();
      processPoints(*msg, wait_ms);
    }
  }

  void processPoints(const sensor_msgs::msg::PointCloud2& msg, double wait_ms) {
    const auto processing_start = std::chrono::steady_clock::now();
    if (msg.header.frame_id.empty() ||
        (msg.header.stamp.sec == 0 && msg.header.stamp.nanosec == 0)) {
      status("rejected: missing frame or timestamp");
      return;
    }
    if (msg.header.frame_id != lidar_frame_) {
      status("rejected: LiDAR frame changed");
      return;
    }
    const auto* x = findField(msg, "x");
    const auto* y = findField(msg, "y");
    const auto* z = findField(msg, "z");
    const auto* t = findField(msg, "time");
    if (msg.is_bigendian || !validField(msg, x) || !validField(msg, y) ||
        !validField(msg, z) || (t && !validField(msg, t)) ||
        msg.point_step == 0 || msg.row_step < msg.width * msg.point_step ||
        msg.data.size() < static_cast<size_t>(msg.height) * msg.row_step) {
      status("rejected: invalid PointCloud2 layout");
      return;
    }
    Cloud scan;
    std::vector<double> point_times;
    scan.reserve(static_cast<size_t>(msg.width) * msg.height);
    if (t) point_times.reserve(scan.capacity());
    const double stamp = rclcpp::Time(msg.header.stamp).seconds();
    for (uint32_t row = 0; row < msg.height; ++row) {
      for (uint32_t col = 0; col < msg.width; ++col) {
        const uint8_t* ptr = msg.data.data() + row * msg.row_step + col * msg.point_step;
        const Point p(readNumber(ptr, *x), readNumber(ptr, *y), readNumber(ptr, *z));
        if (!p.allFinite() || p.norm() < 0.3 || p.norm() > max_range_) continue;
        if (t) {
          const double relative_time = readNumber(ptr, *t);
          if (!std::isfinite(relative_time) || relative_time < 0 || relative_time > 2.0) continue;
          point_times.push_back(stamp + relative_time);
        }
        scan.push_back(p);
      }
    }
    if (scan.empty()) { status("rejected: no valid points"); return; }

    std::string deskew_state;
    if (!t) {
      deskew_state = "off_missing_point_time";
      RCLCPP_WARN_ONCE(get_logger(),
          "PointCloud2 has no per-point time: rotation deskew disabled; no times are synthesized");
    } else {
      std::vector<GyroSample> imu(imu_.begin(), imu_.end());
      std::string reason;
      if (deskewRotationOnly(scan, point_times, imu, stamp, &reason))
        deskew_state = "rotation_only";
      else
        deskew_state = "off_" + reason;
    }
    scan = voxelDownsample(scan, scan_leaf_);
    if (scan.size() < 30) { status("rejected: too few points after voxel filter"); return; }
    if (scan.size() > static_cast<size_t>(max_scan_points_)) {
      // Voxel output is ordered spatially; select evenly over its whole extent.
      // Truncating the first N points would discard much of the field of view.
      Cloud bounded;
      bounded.reserve(max_scan_points_);
      for (int i = 0; i < max_scan_points_; ++i) {
        const size_t index = (static_cast<size_t>(i) * scan.size()) /
                             static_cast<size_t>(max_scan_points_);
        bounded.push_back(scan[index]);
      }
      scan.swap(bounded);
    }

    bool tracking_ok = true;
    std::string state = "initialized";
    std::string seed_state = "first_scan";
    if (initialized_) {
      Eigen::Isometry3d initial_guess = pose_;
      if (last_successful_stamp_ && stamp > *last_successful_stamp_) {
        const std::vector<GyroSample> imu(imu_.begin(), imu_.end());
        Eigen::Matrix3d gyro_delta;
        if (integrateGyroRotation(imu, *last_successful_stamp_, stamp, gyro_delta)) {
          initial_guess.linear() = pose_.linear() * gyro_delta;
          seed_state = "gyro";
        } else {
          seed_state = "last_pose";
        }
      }
      const IcpResult match = matchPointToPoint(scan, local_map_, initial_guess);
      tracking_ok = match.success;
      if (tracking_ok) pose_ = match.pose;
      state = tracking_ok ? "matched" : "tracking_failed:" + match.reason;
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
          "ICP %s: correspondences=%d rmse=%.3f m iterations=%d scan=%zu map=%zu points",
          state.c_str(), match.correspondences, match.rmse, match.iterations,
          scan.size(), local_map_.size());
    } else {
      initialized_ = true;
    }

    if (tracking_ok) last_successful_stamp_ = stamp;

    if (tracking_ok && shouldInsertKeyframe()) {
      Cloud keyframe = transformCloud(scan, pose_);
      if (!global_map_->add(keyframe))
        RCLCPP_WARN_ONCE(get_logger(),
            "Global display map reached max_global_voxels; new cells are being discarded");
      keyframes_.push_back(std::move(keyframe));
      if (keyframes_.size() > static_cast<size_t>(max_keyframes_)) keyframes_.pop_front();
      last_keyframe_pose_ = pose_;
      Cloud all;
      for (const Cloud& keyframe : keyframes_) all.insert(all.end(), keyframe.begin(), keyframe.end());
      local_map_ = voxelDownsample(all, map_leaf_);
      std_msgs::msg::Header map_header = msg.header;
      map_header.frame_id = "demo_map";
      map_pub_->publish(cloudMsg(local_map_, map_header));
      publishGlobalMap(msg.header.stamp);
    }
    publishPose(msg.header.stamp, tracking_ok);
    const double processing_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - processing_start).count();
    status(state + ";deskew=" + deskew_state + ";seed=" + seed_state + ";scan_points=" +
           std::to_string(scan.size()) + ";map_points=" +
           std::to_string(local_map_.size()) + ";global_map_points=" +
           std::to_string(global_map_->size()) + ";wait_ms=" +
           std::to_string(wait_ms) + ";processing_ms=" +
           std::to_string(processing_ms) + ";latency_ms=" +
           std::to_string(wait_ms + processing_ms));
  }

  void publishGlobalMap(const builtin_interfaces::msg::Time& stamp) {
    std_msgs::msg::Header header;
    header.stamp = stamp;
    header.frame_id = "demo_map";
    global_map_pub_->publish(cloudMsg(global_map_->points(), header));
  }

  bool shouldInsertKeyframe() const {
    if (keyframes_.empty()) return true;
    const Eigen::Isometry3d delta = last_keyframe_pose_.inverse() * pose_;
    const double angle = Eigen::AngleAxisd(delta.linear()).angle();
    return delta.translation().norm() >= keyframe_distance_ || angle >= keyframe_angle_;
  }

  void publishPose(const builtin_interfaces::msg::Time& stamp, bool tracking_ok) {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "demo_map";
    // A dedicated estimated frame avoids a second parent for the robot's real lidar_link.
    odom.child_frame_id = kEstimatedPoseFrame;
    odom.pose.pose = poseMsg(pose_);
    // Display-only heuristic; this demo does not estimate a pose covariance.
    // A failure holds the last estimate and uses a larger placeholder variance.
    const double variance = tracking_ok ? 0.1 : 1e3;
    for (int i = 0; i < 6; ++i) odom.pose.covariance[i * 6 + i] = variance;
    // Twist is not estimated; leave numerical fields zero and flag unusable covariance.
    for (int i = 0; i < 6; ++i) odom.twist.covariance[i * 6 + i] = 1e6;
    odom_pub_->publish(odom);
    geometry_msgs::msg::TransformStamped estimated_tf;
    estimated_tf.header = odom.header;
    estimated_tf.child_frame_id = odom.child_frame_id;
    estimated_tf.transform.translation.x = odom.pose.pose.position.x;
    estimated_tf.transform.translation.y = odom.pose.pose.position.y;
    estimated_tf.transform.translation.z = odom.pose.pose.position.z;
    estimated_tf.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster_->sendTransform(estimated_tf);
    if (tracking_ok) {
      geometry_msgs::msg::PoseStamped sample;
      sample.header = odom.header;
      sample.pose = odom.pose.pose;
      path_.header.stamp = stamp;
      path_.poses.push_back(sample);
      if (path_.poses.size() > 3000) path_.poses.erase(path_.poses.begin());
      path_pub_->publish(path_);
    }
  }

  double scan_leaf_, map_leaf_, global_leaf_, max_range_, keyframe_distance_, keyframe_angle_;
  int max_keyframes_, max_scan_points_, max_global_voxels_;
  bool initialized_ = false;
  std::string lidar_frame_, imu_frame_;
  std::optional<Eigen::Matrix3d> imu_to_lidar_;
  std::optional<double> last_successful_stamp_;
  std::deque<GyroSample> imu_;
  std::deque<PendingScan> pending_scans_;
  Eigen::Isometry3d pose_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d last_keyframe_pose_ = Eigen::Isometry3d::Identity();
  std::deque<Cloud> keyframes_;
  Cloud local_map_;
  std::unique_ptr<BoundedVoxelMap> global_map_;
  nav_msgs::msg::Path path_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::TimerBase::SharedPtr pending_timer_;
  rclcpp::TimerBase::SharedPtr global_map_timer_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_map_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
};

}  // namespace amr_3d_demo

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<amr_3d_demo::DemoNode>());
  rclcpp::shutdown();
  return 0;
}
