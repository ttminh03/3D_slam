# Lộ trình khởi đầu

Lộ trình này là thứ tự làm việc đề xuất. Chỉ chốt chi tiết sau khi có yêu cầu thực tế và dữ liệu phù hợp.

Nền tảng giai đoạn đầu: **ROS trên nhánh `dev`**. Phiên bản ROS và cách tổ chức package sẽ được chốt ở Mốc 0.

Robot mục tiêu ban đầu là **AMR dùng LiDAR 3D và IMU**. Mô hình mẫu nằm trong `src/amr_description/`; kích thước, đặc tính cảm biến và phiên bản Gazebo vẫn cần chốt.

## Mốc 0 — Xác định bài toán

- [ ] Chốt tốc độ, định dạng dữ liệu và thông số của LiDAR 3D + IMU đã chọn.
- [ ] Xác định đầu ra: quỹ đạo, bản đồ điểm, mesh hoặc biểu diễn khác; yêu cầu thời gian thực.
- [ ] Chọn phiên bản ROS, môi trường chạy, ngôn ngữ lập trình và giới hạn phần cứng.
- [ ] Chọn bộ dữ liệu phát triển, bộ dữ liệu đánh giá và cách quản lý phiên bản dữ liệu.
- [ ] Chốt chỉ số đo, quy tắc căn chỉnh quỹ đạo và mục tiêu chất lượng ban đầu.

**Hoàn thành khi:** có tài liệu yêu cầu và quyết định kỹ thuật đầu tiên, đủ để dựng một baseline đo được.

## Mốc 1 — Baseline tái lập được

- [ ] Thiết lập build, kiểm tra tự động và hướng dẫn chạy.
- [ ] Nạp dữ liệu, đồng bộ thời gian và kiểm tra hệ tọa độ.
- [ ] Chạy một baseline tối giản từ đầu vào tới quỹ đạo hoặc bản đồ.
- [ ] Lưu cấu hình và báo cáo chỉ số trên bộ đánh giá cố định.

**Hoàn thành khi:** người khác có thể chạy lại và thu được kết quả trong sai số đã công bố.

## Mốc 2 — SLAM 3D đầy đủ

- [ ] Tích hợp front-end ước lượng chuyển động.
- [ ] Tích hợp back-end tối ưu hóa và đóng vòng khi bài toán cần.
- [ ] Cập nhật bản đồ, xuất kết quả và đánh giá từng mô-đun so với baseline.

**Hoàn thành khi:** hệ thống xử lý chuỗi dữ liệu từ đầu đến cuối, có số liệu chất lượng và thời gian chạy.

## Mốc 3 — Độ bền và vận hành

- [ ] Kiểm tra dữ liệu thiếu, chuyển động nhanh, môi trường ít đặc trưng và phiên chạy dài.
- [ ] Đo tài nguyên, xử lý lỗi và hoàn thiện tài liệu triển khai.

**Hoàn thành khi:** giới hạn hoạt động được đo và công bố, các lỗi quan trọng có cách tái hiện.
