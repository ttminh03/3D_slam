# Chạy thử Cartographer 3D gốc cho AMR

Package này chỉ chứa cấu hình và launch để chạy **bản Cartographer gốc** trên mô phỏng AMR. ROS 2 Humble là môi trường thử hiện tại, chưa phải lựa chọn nền tảng cuối cùng.

## Sửa lỗi `package 'cartographer_ros' not found`

Lần thử này dùng Cartographer giải nén ở `/tmp/cartographer_overlay`; gói chưa được cài vào `/opt/ros/humble`. Trong **mỗi terminal mới** chạy Cartographer, nạp overlay trước workspace:

```bash
source /opt/ros/humble/setup.bash
export AMENT_PREFIX_PATH=/tmp/cartographer_overlay/opt/ros/humble:$AMENT_PREFIX_PATH
export LD_LIBRARY_PATH=/tmp/cartographer_overlay/opt/ros/humble/lib:/tmp/cartographer_overlay/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
export PYTHONPATH=/tmp/cartographer_overlay/opt/ros/humble/local/lib/python3.10/dist-packages:$PYTHONPATH
source install/setup.bash
ros2 pkg prefix cartographer_ros
ros2 launch amr_cartographer cartographer_3d.launch.py
```

Lệnh `ros2 pkg prefix` phải in `/tmp/cartographer_overlay/opt/ros/humble`. Nếu thư mục `/tmp/cartographer_overlay` đã mất, dựng lại theo [các lệnh tải và giải nén bên dưới](#cách-chạy-trong-container-phiên-thử-2026-11-12). Dòng `InvalidFrontendLaunchFileError` đi kèm lỗi thiếu package là thông báo tổng quát của ROS launch, không chứng minh file Python sai cú pháp.

## Cấu hình đầu vào

| Đầu vào | Topic / frame | Ghi chú |
| --- | --- | --- |
| LiDAR 3D | `/points` / `lidar_link` | Một `PointCloud2`; không dùng intensity trong scan matching. |
| IMU | `/imu/data` / `imu_link` | `tracking_frame=imu_link`; IMU đo trọng lực khi đứng yên. |
| TF | `map→odom→base_footprint→base_link→{lidar_link,imu_link}` | Cartographer phát `map→odom`; Gazebo phát `odom→base_footprint`; robot_state_publisher phát các cạnh còn lại. |

`use_odometry=false`: `/odom` được dùng qua TF để đặt frame xuất bản nhưng không cấp như phép đo odometry cho thuật toán. Lua tích lũy 5 đám mây cho một lần xử lý; các tham số scan matching và submap khác kế thừa mặc định từ gói Cartographer đang chạy. Các giá trị này là cấu hình thử, chưa tối ưu.

## Chạy lại

Trên máy **đã cài** `cartographer`, `cartographer_ros`, `cartographer_ros_msgs` cho ROS 2 đang thử (không cần overlay `/tmp`):

```bash
source /opt/ros/humble/setup.bash
ros2 pkg prefix cartographer_ros
colcon build --packages-select amr_cartographer --symlink-install
source install/setup.bash
ros2 launch amr_cartographer cartographer_3d.launch.py
```

Ở terminal khác, source cùng môi trường và phát bag khi Gazebo đã dừng:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 bag play /tmp/carto3d_amr_2026-11-12 -r 0.5
```

Sau khi bag phát xong, kiểm tra và lưu trạng thái:

```bash
ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory '{trajectory_id: 0}'
ros2 service call /trajectory_query cartographer_ros_msgs/srv/TrajectoryQuery '{trajectory_id: 0}'
mkdir -p outputs/cartographer_3d_smoke_2026-11-12
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState '{filename: "outputs/cartographer_3d_smoke_2026-11-12/amr_3d.pbstream", include_unfinished_submaps: true}'
```

Bag và `outputs/` không được đưa vào Git. Khi phát lại, các terminal phải dùng cùng `ROS_DOMAIN_ID`. Không chạy Gazebo đồng thời với bag để tránh hai nguồn cùng phát `/points`, IMU và TF.

Bag trong `/tmp` sẽ mất khi môi trường bị dọn. Để ghi một bag mới, làm theo [hướng dẫn mô phỏng AMR](../amr_description/README.md), chạy Gazebo ở một terminal, rồi ở terminal đã source ROS chạy:

```bash
ros2 bag record /clock /points /imu/data /tf /tf_static /odom -o /tmp/carto3d_amr_new
```

Điều khiển robot sau khi bắt đầu ghi và dừng bản ghi bằng `Ctrl+C`. Tuyến di chuyển của bag thử 2026-11-12 chưa được tự động hóa, nên bag mới không có cùng checksum và kết quả 43 pose không phải số mục tiêu của lần chạy mới.

### Cách chạy trong container phiên thử 2026-11-12

Container không có quyền `sudo` và ban đầu chưa cài Cartographer. Đã tải các gói nhị phân sau vào `/tmp`, giải nén để chạy từ user space:

```bash
cd /tmp
apt download ros-humble-cartographer ros-humble-cartographer-ros ros-humble-cartographer-ros-msgs liblua5.2-0
mkdir -p /tmp/cartographer_overlay
for deb in /tmp/ros-humble-cartographer*.deb /tmp/liblua5.2-0_*.deb; do
  dpkg-deb -x "$deb" /tmp/cartographer_overlay
done
source /opt/ros/humble/setup.bash
export AMENT_PREFIX_PATH=/tmp/cartographer_overlay/opt/ros/humble:$AMENT_PREFIX_PATH
export LD_LIBRARY_PATH=/tmp/cartographer_overlay/opt/ros/humble/lib:/tmp/cartographer_overlay/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
export PYTHONPATH=/tmp/cartographer_overlay/opt/ros/humble/local/lib/python3.10/dist-packages:$PYTHONPATH
cd /path/to/3D_slam
colcon build --packages-select amr_cartographer --symlink-install
source install/setup.bash
```

Chạy các lệnh `ros2 launch`, service và `ros2 bag play` ở các terminal đã source môi trường này. Gói `cartographer` core nhị phân không đăng ký trong ament index; launch tìm Lua mặc định trong các ROS prefix và tạo thư mục tạm có symlink để `include` hoạt động.

## Kết quả thử 2026-11-12

- Gói chạy: `ros-humble-cartographer` `2.0.9004-1jammy.20260226.010834`, `ros-humble-cartographer-ros` `2.0.9002-1jammy.20260915.225849`, `ros-humble-cartographer-ros-msgs` `2.0.9002-1jammy.20260907.211210`. Bag `/tmp/carto3d_amr_2026-11-12` có 557 cloud và 2785 IMU trong khoảng 27,8 s thời gian mô phỏng. SHA256 của tệp `.db3`: `178ad4f6b1fd45a4fad0d58539b0c4036d4e4b0d1d5ba26dcd32b659aa21228a`. Lần chạy dùng `ROS_DOMAIN_ID=83`.
- Node tạo trajectory ID 0; log báo `Inserted submap (0, 0)`. `/submap_list` trả về submap ID `(0, 0)` ở frame `map`. `tf2_echo map odom` nhận biến đổi đang cập nhật. Dữ liệu đầu vào được Cartographer thống kê gần 20 Hz cloud và 100 Hz IMU.
- `finish_trajectory` trả trạng thái 0; `trajectory_query` trả **43 pose**, từ khoảng `(0,001; 0; 0,002)` đến `(2,260; 0,014; 0,163)` m trong frame bản đồ; `write_state` trả trạng thái 0 và ghi `outputs/cartographer_3d_smoke_2026-11-12/amr_3d.pbstream` (533 KiB). Log gốc được sao lưu tại `outputs/cartographer_3d_smoke_2026-11-12/cartographer_node.log` (thư mục bị Git bỏ qua).
- Chỉ có **một submap đang hoạt động**; mặc định `submaps.num_range_data=160`, trong khi bag ngắn chỉ sinh 43 node. Log kết thúc báo **0 phép tính ràng buộc ngoài submap**. Chưa có bằng chứng đóng vòng hoặc đánh giá sai số ATE/RPE; không dùng `/odom` bánh xe như ground truth.
- `PointCloud2` mô phỏng chỉ có `x,y,z,intensity`, thiếu `time` và `ring`. Bộ chuyển đổi của wrapper gán thời gian mỗi điểm bằng 0 khi thiếu `time`, nên lần thử này **không chứng minh khả năng khử méo trong lúc quét**. Cần thử bag dài hơn, có vòng lặp và dữ liệu thời gian điểm phù hợp trước khi đánh giá chất lượng SLAM.

`colcon build --packages-select amr_cartographer --symlink-install` đạt; đây là smoke test tích hợp, chưa phải baseline chất lượng.
