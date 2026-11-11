# 0002 — AMR với LiDAR 3D và IMU

## Bối cảnh

Cần một mô hình robot ban đầu để phát triển SLAM 3D trên ROS và chuẩn bị môi trường Gazebo.

## Phương án

- AMR với LiDAR 3D và IMU.
- Một loại robot hoặc tổ hợp cảm biến khác.

## Quyết định

Dùng AMR với LiDAR 3D và IMU cho giai đoạn đầu. Tạo mô hình hình học trong `src/amr_description/` với kích thước mẫu có thể chỉnh sửa.

## Hệ quả

Kích thước robot và thông số cảm biến hiện là giá trị thử nghiệm, chưa đại diện cho phần cứng thật. Package đang chạy thử trên ROS 2 Humble với Gazebo Classic; đây chưa phải quyết định chốt phiên bản nền tảng. Mô hình Gazebo phát `/points` từ LiDAR 3D ở tần số cấu hình 20 Hz, `/imu/data` ở 100 Hz, và nhận `/cmd_vel` để điều khiển bánh vi sai. Các giá trị này cần được đối chiếu với robot thật trước khi dùng để đánh giá SLAM.
