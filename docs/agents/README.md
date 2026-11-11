# Vai trò agent trong dự án

Đây là hồ sơ công việc để agent điều phối giao nhiệm vụ trong các buổi làm việc. Các vai trò được gọi theo việc, không phải tiến trình chạy thường trực. Agent điều phối chịu trách nhiệm ghép kết quả, xử lý xung đột và trả lời người dùng. Mỗi lần giao việc phải nêu mục tiêu, phạm vi tệp, tiêu chí hoàn thành và đầu ra mong đợi. Trong khi agent phụ chạy, người dùng vẫn trò chuyện với agent điều phối ở cuộc chat này; agent điều phối tiếp tục xử lý câu hỏi hoặc việc độc lập và chuyển thông tin mới cho agent phụ nếu có liên quan.

| Vai trò | Phạm vi chính | Hồ sơ |
| --- | --- | --- |
| Agent 1 — Robot và Gazebo | Robot, cảm biến, thế giới mô phỏng và cách chạy | [robot-gazebo.md](robot-gazebo.md) |
| Agent 2 — Nghiên cứu SLAM 3D | Tìm, kiểm chứng và tổng hợp tài liệu | [slam-research.md](slam-research.md) |
| Agent 3 — Code và test | Mã nguồn chính, kiểm thử, sửa lỗi | [code-test.md](code-test.md) |
| Agent 4 — Quản lý dự án | Tiến độ, code, tài liệu, môi trường, báo cáo ngày | [project-manager.md](project-manager.md) |
| Agent 5 — Giải đáp và đối chiếu | Đọc lại Markdown của agent 4, trả lời và xác minh nghi vấn | [review-qa.md](review-qa.md) |

## Luồng phối hợp

1. Agent 4 theo dõi mục tiêu, trạng thái và các quyết định; giao hoặc đề xuất việc cho agent 1–3 qua agent điều phối.
2. Agent 2 cung cấp tài liệu có nguồn và đánh dấu điều chưa chắc. Agent 1 và 3 dùng tài liệu đó để chọn cách triển khai, nhưng phải ghi lại kết quả chạy thực tế.
3. Agent 1 sở hữu phần robot và Gazebo; agent 3 sở hữu thuật toán SLAM, tích hợp và kiểm thử. Khi cần sửa cùng tệp, agent điều phối chỉ định một người sửa, người còn lại phản hồi hoặc kiểm tra.
4. Agent 4 cập nhật báo cáo trong `docs/work_logs/YYYY-MM-DD/WORK_LOG.md` và mục lục `docs/WORK_LOG.md` khi người dùng ra tín hiệu “nghỉ”. Báo cáo phân biệt việc đã làm, đã kiểm chứng và còn vướng; việc tồn đọng được xếp `critical`, `warn` hoặc `info`.
5. Agent 5 đọc lại báo cáo và tài liệu agent 4 để trả lời câu hỏi. Nếu thiếu căn cứ hoặc thấy mâu thuẫn, agent 5 được hỏi lại agent 1–4, rồi nêu rõ bằng chứng và mức độ chắc chắn. Với vị trí cụ thể người dùng nghi ngờ, chỉ mở rộng kiểm tra lại khi người dùng cho phép; không tự sửa báo cáo thay agent 4.

Agent phụ có thể không còn hoạt động ở buổi sau. Tài liệu trong repo là nguồn bàn giao; agent điều phối có thể gọi lại vai trò tương ứng khi có nhiệm vụ mới.
