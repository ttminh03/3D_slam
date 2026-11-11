# Nhật ký 2026-11-11

### Đã thay đổi

- Khởi tạo quy trình trong `AGENTS.md`, `README.md`, `docs/WORKFLOW.md`, `docs/ROADMAP.md`, mẫu issue/PR và `.gitignore`: dùng ROS trên nhánh `dev`, ghi quyết định kỹ thuật và chỉ cập nhật nhật ký khi có tín hiệu “nghỉ”. Tạo hồ sơ năm vai trò tại `docs/agents/` và hai quyết định về nhánh phát triển, AMR dùng LiDAR 3D + IMU.
- Viết `docs/research/3d-slam-overview.md`: giải thích đầu vào, luồng xử lý và so sánh LOAM, LeGO-LOAM, LIO-SAM, FAST-LIO2, Point-LIO, KISS-ICP, Cartographer 3D, RTAB-Map kèm nguồn gốc; đề xuất các baseline để thử, chưa chọn thuật toán chính.
- Tạo package ROS 2 `src/amr_description/`: URDF AMR vi sai với hai bánh chủ động, caster dạng cầu cố định phía trước và phía sau, LiDAR 3D và IMU; launch Gazebo Classic và hướng dẫn chạy. LiDAR được tăng từ 10 lên **20 Hz** trong cấu hình, IMU cấu hình 100 Hz.
- Tạo `worlds/demo.world` rộng **40 × 40 m**, có vùng spawn trống, tường bao và tường hành lang cao **4 m**, kệ, thùng và cột ở nhiều cao độ. Bổ sung hướng dẫn source `/usr/share/gazebo/setup.sh` để khắc phục lỗi tài nguyên shader của `gzclient` trên máy thử nghiệm.
- Chia thay đổi đã thực hiện thành hai commit trên `dev`: `e14a2de` (quy trình/cấu hình/tài liệu) và `3f109d2` (robot/world mô phỏng). Hai commit này đã có trên `origin/dev`; mục nhật ký này được ghi sau thời điểm commit.

### Đã kiểm chứng

- `check_urdf` đọc thành công mô hình; `gz sdf -p` kiểm tra chuyển đổi URDF sang SDF và thấy collision của hai caster. `gz sdf -k` kiểm tra cú pháp world thành công; cú pháp `sim.launch.py` và `colcon build --packages-select amr_description --symlink-install` đều đạt.
- Chạy Gazebo không có giao diện: robot spawn thành công; có các topic `/points` (`PointCloud2`), `/imu/data` (`Imu`), `/odom`, `/cmd_vel`, `/clock`, `/joint_states`. Header dữ liệu dùng `lidar_link` và `imu_link`; gửi lệnh tiến `linear.x=0.2` làm tọa độ odometry `x` tăng trong lần thử. Chưa đo tần số `/points` thực tế ở mức 20 Hz.
- Một lần chạy GUI cô lập sau khi source Gazebo setup không còn lỗi shader. Tuy nhiên log launch do người dùng cung cấp sau đó vẫn cho thấy `gzclient` thoát mã `-6` với lỗi `Camera` assertion, trong khi `gzserver` và robot spawn thành công. Chưa xác nhận GUI chạy ổn định trên phiên launch mới của người dùng.
- Sau lần kiểm tra trước tín hiệu nghỉ, nhánh `dev` sạch tại `3f109d2`. Chưa chạy lại mô phỏng có giao diện sau toàn bộ thay đổi world và tần số LiDAR.

### Việc tiếp theo / còn vướng

- [warn] Điều tra lỗi `gzclient` trên phiên chạy của người dùng; xác nhận biến `GAZEBO_RESOURCE_PATH`, log Gazebo và khả năng mở giao diện sau khi source setup trong cùng terminal.
- [warn] Robot, hai caster cố định, khối lượng và cảm biến hiện là mô hình mẫu; cần thông số robot/cảm biến thật, kiểm tra động lực học và đo tần số dữ liệu thực tế trước khi đánh giá thuật toán.
- [info] Chưa triển khai hoặc đo baseline/thuật toán SLAM 3D; tiếp tục Mốc 0: chốt đầu ra, phiên bản nền tảng, dữ liệu, đồng bộ thời gian LiDAR–IMU, định dạng điểm và tiêu chí đánh giá.
