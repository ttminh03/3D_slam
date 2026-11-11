# Hướng dẫn cộng tác trong repo

- Tuân theo `docs/WORKFLOW.md` khi thay đổi dự án.
- Giai đoạn hiện tại dùng ROS và làm việc trực tiếp trên nhánh `dev`. Chưa tự ý chọn phiên bản ROS hoặc tách nhánh tính năng khi chưa có quyết định mới.
- Khi nhiệm vụ phù hợp và môi trường hỗ trợ agent phụ, điều phối theo năm vai trò trong `docs/agents/README.md`. Giao đúng phạm vi và yêu cầu đầu ra cụ thể; không giả định các agent chạy thường trực hoặc có thể chạy đồng thời cả năm vai trò.
- Sau khi giao việc cho agent phụ, tiếp tục trao đổi và xử lý các yêu cầu độc lập của người dùng trong cùng cuộc chat. Không dùng việc chờ agent như một bước chặn cuộc trò chuyện; kiểm tra kết quả khi có cập nhật hoặc khi cần tổng hợp. Tin nhắn mới của người dùng trong lúc agent đang chạy là thông tin để điều chỉnh công việc đang làm, trừ khi người dùng nói rõ muốn dừng hoặc đổi mục tiêu.
- Agent 3 chịu trách nhiệm chính về mã nguồn và kiểm thử. Agent 1 quản lý robot và mô phỏng Gazebo, phối hợp với agent 3 khi thay đổi mã. Agent 2 nghiên cứu tài liệu SLAM 3D và dẫn nguồn. Agent 4 quản lý tiến độ, tài liệu, môi trường và nhật ký. Agent 5 đọc lại tài liệu do agent 4 viết để giải đáp thắc mắc; được hỏi lại các agent khác khi thiếu căn cứ.
- Với phần người dùng nghi ngờ, agent 5 chỉ mở rộng kiểm tra lại vị trí hoặc bằng chứng cụ thể đó khi người dùng cho phép. Nếu chưa có quyền kiểm tra, nêu rõ giới hạn của câu trả lời.
- Khi người dùng ra tín hiệu “nghỉ”, trước khi kết thúc buổi làm việc hãy cập nhật `docs/WORK_LOG.md` theo ngày địa phương `Asia/Ho_Chi_Minh`. Ghi tất cả thay đổi thực tế từ lần tổng kết trước, kết quả kiểm chứng và việc còn vướng. Bổ sung vào mục ngày hiện có nếu cần; không ghi công việc chưa làm như đã hoàn thành.
- Chú ý ghi lại những gì còn tồn đọng lại, phân loại theo mức độ nghiêm trọng: critical, warn, và info nhé
- Khi chưa có tín hiệu “nghỉ”, tiếp tục làm việc bình thường; không coi mọi câu kết thúc hội thoại là tín hiệu chốt nhật ký.
