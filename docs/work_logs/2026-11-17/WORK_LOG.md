# Nhật ký 2026-11-17

Tổng kết các thay đổi thực tế từ lần chốt ngày 2026-11-11; các commit liên quan được tạo ngày 12, 13 và 17/11 trên nhánh `dev`.

### Đã thay đổi

- **12/11 — `bf73ad2`:** thêm package cấu hình và launch `src/amr_cartographer/` để chạy Cartographer 3D gốc với `/points`, `/imu/data` và TF của AMR; viết hướng dẫn tái chạy và kết quả smoke test trong README package. Thêm `docs/research/cartographer-3d-roadmap.md`, trang nhập môn LIO-SAM `docs/research/lio-sam-nhap-mon.html` và liên kết hướng nghiên cứu Cartographer trong `docs/ROADMAP.md`. Cấu hình URDF LiDAR từ 20 lên **25 Hz**, IMU từ 100 lên **400 Hz**; đây là giá trị cấu hình, không phải tần số thực tế đã đo sau sửa.
- **13/11 — `a4ae6e3`:** thêm quyết định `docs/decisions/0003-demo-lidar-odometry-3d.md` và package `src/amr_3d_demo/` gồm node, lõi thuật toán, launch, README và bài kiểm thử. Demo nhận cloud 3D và IMU; dùng deskew quay khi có thời gian từng điểm, gyro để khởi tạo hướng, ICP điểm–điểm với local map tối đa 8 keyframe, map điểm tích lũy `/demo/map` và các topic pose/path/status. Thêm dàn ý nghiên cứu sâu `docs/research/lio-sam-nghien-cuu-sau.md` và cập nhật trang nhập môn. Demo chưa triển khai đặc trưng cạnh/mặt, graph IMU, đóng vòng hay LIO-SAM hoàn chỉnh.
- **17/11 — `e90c3f7`:** thêm hai bản HTML `docs/research/lio-sam-cong-thuc.html` và `docs/research/lio-sam-nghien-cuu-sau.html`, cập nhật liên kết trong `docs/research/lio-sam-nhap-mon.html` để đọc giáo trình/công thức.
- **Buổi trao đổi 17/11:** phân tích cách chọn đặc trưng, keyframe, local map, các topic hiển thị trong RViz, khoảng cách điểm–đường/điểm–mặt, Jacobian và số điểm tạo ràng buộc của LIO-SAM; đối chiếu với demo ICP điểm–điểm. Buổi trao đổi này **không sửa mã nguồn hoặc cấu hình**.

### Đã kiểm chứng

- Theo kết quả ghi trong `src/amr_cartographer/README.md`, package Cartographer build được và smoke test trên bag ngày 12/11 tạo trajectory ID 0, **43 pose**, một submap đang hoạt động, TF `map→odom` và file `.pbstream`. Bag có 557 cloud, 2785 IMU; phiên chạy được ghi nhận gần **20 Hz cloud, 100 Hz IMU**. Đây là dữ liệu của bag thử, không phải phép đo lại cấu hình URDF 25/400 Hz.
- Theo `src/amr_3d_demo/README.md`, các lượt Gazebo headless ngắn ngày 13/11 báo `matched` 127/127 scan khi quay tại chỗ và 155/155 scan khi đi thẳng rồi quay; sai khác góc với wheel odometry lần lượt khoảng 0,006 rad và 0,004 rad, sai khác dịch chuyển LiDAR sau tính offset khoảng 0,088 m và 0,097 m. Độ trễ trung vị lượt đi thẳng rồi quay là 73,63 ms/scan. README còn ghi kết quả hai lượt thử ban đầu trước sửa TF/map. Đây là kết quả đã ghi từ thử nghiệm trước, **không phải phép thử chạy lại ngày 17/11**; wheel odometry không phải ground truth độc lập.
- `src/amr_3d_demo/test/test_algorithm.cpp` có bài kiểm thử cho deskew quay, vùng phủ IMU, dự đoán gyro, khôi phục phép biến đổi ICP, từ chối hình học suy biến và voxel/map giới hạn bộ nhớ. Trong lần chốt nhật ký này chưa chạy lại `colcon test`, nên không xác nhận trạng thái đạt của bộ test ở HEAD hiện tại.
- Đối chiếu `git log` và `git status`: trước khi viết nhật ký, nhánh `dev` sạch, đồng bộ `origin/dev`, HEAD `e90c3f7`. Phần đóng buổi chỉ thêm tệp nhật ký này và liên kết mục lục.

### Việc tiếp theo / còn vướng

- **critical:** Chưa thấy trở ngại chặn việc tiếp tục học và thử nghiệm hiện tại.
- **warn:** Cloud Gazebo hiện thiếu `time` từng điểm và `ring`; chưa kiểm chứng được deskew LiDAR gốc/feature extraction LIO-SAM trên nguồn dữ liệu này. Cần nguồn dữ liệu phù hợp hoặc quyết định sửa mô phỏng rồi kiểm tra hợp đồng dữ liệu.
- **warn:** Cartographer mới có một submap đang hoạt động, không có ràng buộc ngoài submap; demo ICP cũng chưa có đóng vòng. Chưa có đánh giá ATE/RPE với ground truth độc lập. Bag Cartographer nằm trong `/tmp`, tuyến chạy chưa tái lập tự động.
- **warn:** Demo có lượt xử lý chậm hơn chu kỳ scan; kết quả ngắn chưa chứng minh chạy ổn định ở tần số LiDAR cấu hình 25 Hz. Cần đo lại tần số thực tế, độ trễ theo phân vị và tracking trên lộ trình dài. Lỗi GUI `gzclient` đã ghi ngày 11/11 chưa có bằng chứng xác nhận đã hết.
- **info:** Chưa chốt ROS distro, cảm biến/robot thật, bộ dữ liệu chuẩn và ngưỡng chấp nhận thuật toán. Tiếp tục học từng khối LIO-SAM và phân biệt local feature map với cloud tích lũy hiển thị; chọn một bài kiểm chứng nhỏ trước khi mở rộng demo.
