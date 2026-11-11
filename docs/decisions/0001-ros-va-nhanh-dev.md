# 0001 — ROS và nhánh dev cho giai đoạn đầu

## Bối cảnh

Dự án SLAM 3D đang ở giai đoạn khởi tạo. Chưa chốt cảm biến, thuật toán và cấu trúc phần mềm.

## Phương án

- Phát triển ban đầu trên ROS ở nhánh `dev`.
- Chọn nền tảng và cách chia nhánh khác ngay từ đầu.

## Quyết định

Dùng ROS làm nền tảng giai đoạn đầu và làm việc trực tiếp trên nhánh `dev`. Việc tách nhánh và chọn phiên bản ROS được quyết định sau.

## Hệ quả

Tài liệu và mã nguồn mới cần phù hợp với ROS khi phiên bản được chốt. `main` chỉ nhận nội dung đã kiểm chứng từ `dev`; hiện chưa có mốc tích hợp.
