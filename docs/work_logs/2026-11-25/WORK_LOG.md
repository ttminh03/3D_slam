# Nhật ký 2026-11-25

Tổng kết từ lần chốt ngày 2026-11-17 trên nhánh `dev`.

### Đã thay đổi

- Buổi trao đổi tập trung vào **Bài 9 LIO-SAM**: pose LiDAR là pose sau khớp scan; giải thích ba đại lượng tiền tích phân IMU `ΔR`, `Δv`, `Δp`, hai nhánh tích phân IMU và luồng LiDAR hiệu chỉnh pose rồi đồ thị tối ưu pose, vận tốc, bias để phục vụ dự đoán tiếp theo.
- Làm rõ trong source LIO-SAM gốc, deskew dùng mẫu gyro IMU đã đổi hệ trục nhưng không trừ trực tiếp bias vừa tối ưu; dự đoán pose khởi tạo cho scan matching dùng nhánh tích phân IMU có bias cập nhật. So sánh cách khớp từ một pose khởi tạo, tính sai số cạnh/mặt và Jacobian với correlative scan matching 2D duyệt nhiều pose ứng viên trong cửa sổ tìm kiếm.
- Đối chiếu demo hiện tại: `integrateGyroRotation()` chỉ tích lũy góc quay để khởi tạo ICP; demo chưa có tiền tích phân gia tốc, tối ưu vận tốc/bias hoặc vòng phản hồi pose LiDAR–IMU. **Không sửa mã nguồn, cấu hình hay tài liệu thuật toán trong buổi này.** Phần thay đổi tệp chỉ là nhật ký ngày này và liên kết mục lục.

### Đã kiểm chứng

- Trước khi ghi nhật ký, `git status --short --branch` cho thấy nhánh `dev` sạch và đồng bộ `origin/dev`; `git log --since='2026-11-18 00:00:00 +0700' --oneline` không có commit mới. Vì vậy không ghi nhận thay đổi code từ lần chốt trước.
- Các giải thích đã đối chiếu với luồng xử lý trong source LIO-SAM và mã demo khi trao đổi. **Không chạy build, test, Gazebo hay phép đo thực nghiệm mới** trong buổi này; các kết quả thử nghiệm cũ vẫn theo nhật ký 17/11.

### Việc tiếp theo / còn vướng

- **critical:** Không ghi nhận trở ngại chặn việc tiếp tục nghiên cứu hoặc thử nghiệm.
- **warn:** Demo chưa triển khai tối ưu pose, vận tốc và bias IMU như LIO-SAM; chưa có kiểm chứng định lượng cho lợi ích của dự đoán pose khởi tạo hoặc thời gian scan matching. Nguồn cloud Gazebo thiếu `time` từng điểm và `ring` như đã ghi ngày 17/11.
- **warn:** Chưa có đánh giá ATE/RPE với ground truth độc lập, kiểm tra tracking dài hạn hoặc xác nhận vấn đề GUI Gazebo đã hết; các việc này vẫn còn từ lần chốt trước.
- **info:** Tiếp tục học riêng từng bước tiền tích phân IMU, LiDAR pose factor và tối ưu bias trước khi quyết định mở rộng demo. Chưa chốt ROS distro, cảm biến/robot thật, bộ dữ liệu chuẩn và ngưỡng chấp nhận thuật toán.
