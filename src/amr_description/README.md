# Mô hình AMR mẫu

`urdf/amr.urdf` mô tả khung xe vi sai, hai bánh chủ động, hai điểm tựa caster ở đầu và đuôi xe, LiDAR 3D và IMU. Package ROS 2 này có cấu hình **chạy thử** trên ROS 2 Humble + Gazebo Classic 11 đang có trong môi trường hiện tại; đây chưa phải quyết định chốt phiên bản nền tảng của dự án.

## Quy ước và kích thước giả định

- Đơn vị: mét, kilogram, radian; trục `x` hướng tiến, `y` sang trái, `z` hướng lên.
- `base_footprint` ở mặt đất; `base_link` cao 0,18 m, tại tâm khung 0,55 × 0,42 × 0,18 m.
- Bánh bán kính 0,09 m, dày 0,04 m; tâm bánh cách nhau 0,47 m. Hai bánh đặt trên trục `y` của `base_link` và quay quanh trục `+y`.
- Hai caster hình cầu bán kính 0,05 m ở `x = +0,21 m` (đầu xe) và `x = -0,21 m` (đuôi xe), trên trục giữa `y = 0`; tâm ở `z = 0,05 m` so với đất.
- `lidar_link` ở vị trí `(0,10; 0; 0,17)` so với `base_link`, cao 0,35 m so với đất. `imu_link` ở `(0,04; 0; 0,02)`, cao 0,20 m.
- Khối lượng/quán tính là giá trị mẫu dựa trên hình học đơn giản, cần thay theo robot và cảm biến thật trước khi đánh giá động lực học.

Sơ đồ frame: `base_footprint → base_link → {left_wheel_link, right_wheel_link, front_caster_link, rear_caster_link, lidar_link, imu_link}`. Các khớp cảm biến và caster cố định; khớp bánh xe `continuous`. `front_caster_link` và `rear_caster_link` là hai quả cầu cố định làm điểm tựa hình học, chưa mô phỏng cơ cấu caster lăn tự do.

## Kiểm tra mô hình

Từ gốc repo, nếu có công cụ ROS:

```bash
check_urdf src/amr_description/urdf/amr.urdf
```

## Chạy mô phỏng

Từ gốc repo, trên máy có ROS 2 Humble, `gazebo_ros` và `gazebo_plugins`:

```bash
source /usr/share/gazebo/setup.sh
source /opt/ros/humble/setup.bash
colcon build --packages-select amr_description --symlink-install
source install/setup.bash
ros2 launch amr_description sim.launch.py
```

Gazebo mở `worlds/demo.world`, một sân 40 × 40 m với tường bao cao 4 m. Vùng 6 × 6 m quanh gốc tọa độ trống để spawn robot. Phía đông bắc có hành lang rộng 2,8 m với tường cao 4 m; phía tây nam có lối giữa hai kệ rộng 3 m; phía tây bắc có các thùng với khe hở tối thiểu 2,5 m; phía đông nam có cột tròn nhiều độ cao. Các vật cản khác cao từ 0,6 đến 2,5 m để LiDAR 3D có bề mặt ở nhiều cao độ. Mặt đất ở `z=0`, robot spawn tại gốc như cấu hình launch.

Để chạy không có cửa sổ: `ros2 launch amr_description sim.launch.py gui:=false`. Mở terminal khác, source ba file setup trên rồi kiểm tra:

Nếu `gzserver` chạy nhưng không thấy cửa sổ, kiểm tra terminal có dòng `gzclient ... exit code -6` và log `~/.gazebo/client-*/default.log` có lỗi `Unable to find shader lib`. Trên máy này, lỗi đó xảy ra khi chưa source `/usr/share/gazebo/setup.sh`; chạy lại toàn bộ các lệnh trên trong cùng terminal. Dùng `Ctrl+C` để dừng phiên launch cũ trước khi khởi động lại.

```bash
ros2 topic list
ros2 topic echo /imu/data --once --field header
ros2 topic echo /points --once --field header
ros2 topic echo /odom --once --field pose.pose.position
ros2 topic echo /clock --once
ros2 run tf2_ros tf2_echo base_footprint lidar_link
```

Điều khiển thử robot tiến trong khoảng 2 giây (20 lệnh ở 10 Hz):

```bash
ros2 topic pub -r 10 -t 20 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2}, angular: {z: 0.0}}'
```

Các topic dự kiến: `/points` (`sensor_msgs/msg/PointCloud2`, `lidar_link`, 20 Hz, 360 × 16 tia, 0,15–100 m), `/imu/data` (`sensor_msgs/msg/Imu`, `imu_link`, 100 Hz), `/odom` (`nav_msgs/msg/Odometry`) và `/cmd_vel` (`geometry_msgs/msg/Twist`). `robot_state_publisher` phát `robot_description` và TF cố định; plugin bánh xe phát `/joint_states`; `gazebo_ros_diff_drive` phát `odom → base_footprint`. Đặt `use_sim_time` cho các node tích hợp SLAM.

Đây là thông số cảm biến và khối lượng giả định để kiểm tra pipeline. LiDAR không có nhiễu mô phỏng; hai caster là quả cầu cố định nên tính chất chuyển động chưa đại diện robot thật. Cần thay thông số bằng robot/cảm biến thực trước khi đánh giá SLAM.
