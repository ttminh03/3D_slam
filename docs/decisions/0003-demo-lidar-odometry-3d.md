# 0003 — Demo LiDAR odometry 3D với local map

## Bối cảnh

Mô phỏng AMR đã có `/points` 3D và `/imu/data`, nhưng cloud hiện thiếu `time` và `ring`. Cần một chương trình nhỏ chạy trên dữ liệu này để học scan matching và kiểm tra đường dữ liệu trước khi tái triển khai từng khối LIO-SAM. Theo `docs/WORKFLOW.md`, giả thuyết là một ICP điểm–điểm với local map hữu hạn có thể tạo quỹ đạo ngắn ổn định trong thế giới demo và cho thấy rõ các tình huống mất ràng buộc.

## Phương án đã cân nhắc

- Chạy trực tiếp source LIO-SAM: cần `ring` và thời gian từng điểm; không phù hợp cloud mô phỏng hiện tại.
- Gọi toàn bộ PCL ICP: nhanh triển khai nhưng che phần residual/Jacobian/kiểm tra suy biến đang cần học.
- Tự viết Gauss–Newton ICP điểm–điểm, dùng PCL chỉ cho KD-tree: cho phép đọc và kiểm thử phép cập nhật pose, vẫn đủ nhỏ cho demo.

## Quyết định

Chọn phương án thứ ba trong package ROS 2 `amr_3d_demo` riêng. Input là `/points` và `/imu/data`; `/odom` chỉ dùng so sánh sau căn chỉnh, không đi vào ước lượng. Node chờ IMU tối đa 180 ms theo đồng hồ máy để gyro phủ thời điểm scan, rồi dùng tích phân gyro làm hướng khởi tạo ICP; nếu thiếu thì dùng pose trước. Map cục bộ là các cloud keyframe đã biến đổi về `demo_map`, tối đa 8 keyframe rồi lọc voxel làm target ICP. Map hiển thị `/demo/map` tích lũy keyframe bằng voxel cố định, có giới hạn 30.000 ô mặc định và phát lại mỗi giây để RViz mở muộn nhận được. Đây là point cloud, không phải occupancy map. Pose ước lượng tại vị trí LiDAR xuất qua `/demo/odom` và `/demo/path`; trạng thái và thời gian xử lý qua `/demo/status`. Node phát TF động `demo_map → demo_lidar_pose` từ scan đầu và đặt `odom.child_frame_id=demo_lidar_pose`; dùng child riêng để không đặt parent thứ hai cho `lidar_link` thật của robot.

Deskew quay từ gyro được cài riêng. Nó chỉ chạy khi PointCloud2 có `time` tính theo giây từ `header.stamp` và IMU phủ đủ scan. Dữ liệu mô phỏng hiện thiếu `time` nên đi qua nguyên trạng, có log và status. Không tạo `ring` hoặc `time` giả.

## Hệ quả và kiểm chứng

Demo này kiểm chứng hợp đồng dữ liệu, biến đổi 3D, residual và tối ưu scan-to-map ngắn hạn. Nó không kiểm chứng deskew trên Gazebo hiện tại và không thay thế feature scan matching, graph IMU/pose, loop closure của LIO-SAM. ICP điểm–điểm có thể lệch ở hành lang đối xứng; `/demo/local_map` là cửa sổ ICP, còn `/demo/map` là cloud tích lũy không có loop closure nên quỹ đạo và tường vẫn có thể trôi/lệch khi chạy lâu.

Kiểm tra tự động: quay tổng hợp để kiểm tra deskew, cloud 3D với pose biết trước để kiểm tra ICP, hình học đường thẳng suy biến để kiểm tra từ chối cập nhật. Kiểm tra tích hợp: Gazebo headless, robot đứng yên rồi chạy thẳng và quay, đọc `/demo/status`, `/demo/odom`, số scan matched/failed và `processing_ms`. Số đo phải ghi kèm cấu hình `max_scan_points`, phần cứng và điều kiện mô phỏng; không coi wheel `/odom` là ground truth tuyệt đối.
