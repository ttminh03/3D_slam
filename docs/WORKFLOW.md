# Quy trình quản lý dự án

## 1. Từ ý tưởng đến công việc

- Mỗi việc có một mục theo dõi mô tả mục tiêu, phạm vi, tiêu chí hoàn thành và cách kiểm chứng; dùng issue khi có hệ thống quản lý issue.
- Dùng nhãn `research`, `feature`, `bug`, `experiment`, `docs` theo tính chất công việc; dùng `blocked` khi đang chờ đầu vào.
- Việc thử nghiệm cần ghi giả thuyết, cấu hình, bộ dữ liệu và tiêu chí so sánh với mốc hiện tại.
- Không nhận nhiều việc lớn cùng lúc. Chia việc sao cho mỗi thay đổi có một mục tiêu rõ ràng và có thể kiểm chứng độc lập.

## 2. Nhánh và thay đổi

- `dev` là nhánh làm việc hiện tại. Giai đoạn đầu phát triển trực tiếp trên `dev` với nền tảng ROS; chưa tách nhánh cho từng tính năng.
- `main` dành cho phiên bản đã được kiểm chứng. Chỉ tích hợp từ `dev` khi có nội dung ổn định để phát hành.
- Khi dự án lớn hơn hoặc có nhiều người cùng sửa, tách nhánh từ `dev`: `feature/<ten-ngan>`, `fix/<ten-ngan>`, `experiment/<ten-ngan>` hoặc `docs/<ten-ngan>`. Quy tắc này sẽ được kích hoạt khi nhóm quyết định tách nhánh.
- Commit nhỏ, mô tả bằng động từ và nêu nội dung thay đổi, ví dụ `Add point cloud input validation`.
- Không đưa dữ liệu thô, bản ghi cảm biến, đầu ra lớn, khóa bí mật hoặc đường dẫn cá nhân vào Git. Lưu cách lấy và phiên bản dữ liệu trong tài liệu thử nghiệm.

## 3. Kiểm tra thay đổi và duyệt pull request

Mỗi thay đổi trên `dev` cần nêu mục tiêu liên quan, nội dung chính, cách chạy và kết quả kiểm chứng. Khi dùng pull request, đưa các thông tin này vào mô tả. Người tạo tự kiểm tra trước khi hoàn tất thay đổi:

- Mã biên dịch hoặc chạy được theo hướng dẫn của repo; kiểm tra tự động hiện có đều đạt.
- Thay đổi thuật toán có thí nghiệm tái lập được: phiên bản mã, dữ liệu, tham số, seed nếu có, phần cứng nếu ảnh hưởng đến thời gian chạy.
- Báo cáo chỉ số phù hợp, chẳng hạn ATE/RPE cho quỹ đạo, độ hoàn chỉnh/chính xác bản đồ và thời gian xử lý; nêu rõ cách căn chỉnh quỹ đạo và đơn vị đo.
- Tài liệu sử dụng, cấu hình và quyết định thiết kế được cập nhật khi hành vi thay đổi.
- Nêu giới hạn hoặc trường hợp thất bại đã biết.

Ưu tiên một lượt duyệt từ người khác trước khi gộp vào `main`. Nếu chỉ có một người làm việc, tự duyệt bằng danh sách trên và ghi lại kết quả kiểm chứng. Không gộp khi kiểm tra bắt buộc chưa đạt.

## 4. Quyết định và đánh giá

- Ghi một tài liệu ngắn trong `docs/decisions/` khi chọn cảm biến, biểu diễn bản đồ, framework, định dạng dữ liệu, bộ đánh giá hoặc thay đổi giao diện giữa các mô-đun. Tài liệu nêu bối cảnh, các phương án, quyết định và hệ quả.
- Giữ một baseline đã đo trên cùng bộ dữ liệu và cùng quy trình đánh giá trước khi chấp nhận thay đổi về thuật toán.
- Với lỗi, lưu đầu vào tối thiểu để tái hiện, hành vi mong đợi, hành vi thực tế và bản sửa được kiểm tra bằng gì.

## 5. Nhịp làm việc

Đầu mỗi chu kỳ ngắn: chọn mục tiêu và điều kiện hoàn thành. Trong chu kỳ: cập nhật issue khi có kết quả hoặc trở ngại. Cuối chu kỳ: gộp việc đã kiểm chứng, ghi lại chỉ số so với baseline và chọn việc tiếp theo. Khi dự án còn nhỏ, thực hiện nhịp này theo từng pull request; không cần cuộc họp cố định.

Các việc được chuyển cho [agent theo vai trò](agents/README.md) khi cần. Agent phụ trả lại kết quả cụ thể cho agent điều phối; những người cùng sửa repo phải thống nhất tệp sở hữu để tránh ghi đè. Agent 4 tổng hợp trạng thái dự án, còn agent 5 kiểm tra nội dung báo cáo khi cần giải đáp cho người dùng.

## 6. Nhật ký thay đổi theo ngày

Khi người dùng ra tín hiệu **“nghỉ”**, cập nhật [nhật ký làm việc](WORK_LOG.md) trước khi kết thúc buổi làm việc. Dùng ngày địa phương của người dùng (`Asia/Ho_Chi_Minh`) và ghi những thay đổi kể từ lần tổng kết trước, kể cả mã nguồn, cấu hình, tài liệu và quyết định kỹ thuật. Với mỗi thay đổi, nêu tệp hoặc mô-đun liên quan và kết quả thực tế; không ghi việc mới chỉ dự định làm như đã hoàn thành.

Mỗi mục ngày có ba phần: **Đã thay đổi**, **Đã kiểm chứng**, **Việc tiếp theo / còn vướng**. Phần tồn đọng được phân loại `critical` (chặn công việc hoặc có rủi ro nghiêm trọng), `warn` (cần xử lý nhưng còn đường tiếp tục) và `info` (theo dõi hoặc cải thiện). Nếu cùng một ngày có nhiều buổi làm việc, bổ sung vào mục của ngày đó. Nếu chưa thay đổi mã nguồn, ghi rõ điều này và vẫn ghi các thay đổi khác. Không chờ đến cuối ngày theo đồng hồ; tín hiệu “nghỉ” là thời điểm chốt nhật ký.
