# Nhập môn SLAM 3D cho AMR dùng LiDAR 3D và IMU

> Tài liệu của Agent 2. Đây là tổng quan để chọn **thử nghiệm**, chưa phải quyết định chọn thuật toán. Các nhận định về từng hệ thống dựa vào bài báo hoặc tài liệu của nhóm tác giả ở cuối bài. Phần ghi **đề xuất** là suy luận cho dự án, cần đo trên dữ liệu của AMR.

## SLAM là gì?

**SLAM** (*Simultaneous Localization and Mapping*) là việc robot vừa ước lượng **mình đang ở đâu** vừa xây dựng **bản đồ môi trường chưa biết**. Với AMR, LiDAR 3D đo các điểm trên vật thể xung quanh; IMU đo gia tốc và tốc độ góc. Từ nhiều lần đo liên tiếp, hệ thống ước lượng quỹ đạo của robot và đặt các điểm đo vào cùng một hệ tọa độ. Ví dụ: robot đi một vòng hành lang và quay về cửa ban đầu; bản đồ tốt phải cho hai lần quan sát cửa trùng nhau.

SLAM không đồng nghĩa với tránh vật cản hay lập đường đi. Các chức năng điều hướng có thể dùng đầu ra của SLAM nhưng là bài toán khác. Bản đồ 3D cũng có nhiều dạng: đám mây điểm, các *submap*, lưới chiếm chỗ; không phải thuật toán nào cũng tạo mesh.

## Một vòng xử lý, theo thứ tự dễ hình dung

```text
LiDAR 3D + IMU
    ↓ kiểm tra dấu thời gian, frame và hiệu chuẩn vị trí hai cảm biến
    ↓ khử méo một lượt quét do robot vẫn chuyển động khi LiDAR đo
    ↓ ghép lượt quét mới với lượt trước hoặc bản đồ cục bộ
Vị trí mới (odometry) ──→ thêm quan sát vào bản đồ
    ↓ khi nhận ra nơi từng đi qua: ràng buộc đóng vòng
Tối ưu lại quỹ đạo và bản đồ (nếu hệ thống có back-end toàn cục)
```

1. **Đầu vào:** mỗi điểm LiDAR cần có thời gian đo nếu thuật toán dùng khử méo theo từng điểm; IMU và LiDAR phải đồng bộ, và phải biết phép biến đổi giữa hai frame. Đây là yêu cầu được nêu rõ trong tài liệu [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM#prepare-lidar-data) và [Point-LIO](https://github.com/hku-mars/Point-LIO).
2. **Front-end / odometry:** ước lượng chuyển động giữa các quan sát gần nhau. Kết quả thường mượt, cập nhật nhanh nhưng **trôi** dần vì sai số tích lũy.
3. **Mapping:** dùng vị trí ước lượng để ghép các đám mây điểm thành bản đồ cục bộ hoặc toàn cục. Có bản đồ chưa có nghĩa là sai số toàn cục đã được sửa.
4. **Loop closure / đóng vòng:** khi robot quay lại nơi cũ, hệ thống nhận ra sự trùng lặp, tạo ràng buộc giữa hai thời điểm xa nhau, rồi tối ưu lại các vị trí. Đóng vòng sai có thể làm hỏng bản đồ; cần xác nhận hình học trước khi áp dụng. [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM#other-notes) có ví dụ đóng vòng nhưng nhóm tác giả gọi đó là *proof of concept*.

LiDAR cho hình học ổn định nhưng có thể khó ghép trong hành lang dài, khu vực ít đặc trưng, mặt kính hoặc cảnh có nhiều vật chuyển động. IMU giúp dự đoán chuyển động giữa các lượt quét và khử méo; bản thân IMU cũng trôi theo thời gian. Việc kết hợp hai cảm biến thường gọi là **LIO** (*LiDAR–inertial odometry*). [LOAM (2014)](https://publications.ri.cmu.edu/loam-lidar-odometry-and-mapping-in-real-time) và [LIO-SAM (2020)](https://arxiv.org/abs/2007.00258) minh họa hai hướng tiếp cận này.

## Các họ thuật toán nên biết

| Hệ thống | Ý tưởng chính, đầu vào | Có đóng vòng trong hệ thống được mô tả? | Điểm cần lưu ý với AMR này |
| --- | --- | --- | --- |
| **LOAM** (2014) | Chia ước lượng chuyển động LiDAR nhanh và ghép bản đồ chậm, chính xác hơn; bài gốc không cần IMU. [Bài gốc](https://publications.ri.cmu.edu/loam-lidar-odometry-and-mapping-in-real-time) | Bài gốc tập trung odometry + mapping, không trình bày back-end đóng vòng như một thành phần chính. | Nên đọc để hiểu cấu trúc nền tảng; không tự động tận dụng IMU đã có. |
| **LeGO-LOAM** (2018) | Biến thể LiDAR cho xe mặt đất, phân đoạn mặt đất trước khi trích đặc trưng; IMU tùy chọn. [Repo tác giả](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM) | Có phần đóng vòng trong hệ thống; cần kiểm chứng trên dữ liệu thực. | Repo nêu giả thiết mặt đất luôn nhìn thấy, LiDAR VLP-16 nằm ngang; khó áp thẳng sang cảm biến khác. |
| **LIO-SAM** (2020) | LiDAR + IMU kết hợp chặt qua *factor graph*; IMU hỗ trợ khử méo; có thể thêm ràng buộc GPS và đóng vòng. [Bài báo](https://arxiv.org/abs/2007.00258), [repo tác giả](https://github.com/TixiaoShan/LIO-SAM) | Có cơ chế, nhưng ví dụ đóng vòng trong repo chỉ là minh họa. | Bản gốc yêu cầu trường `time`, `ring`, LiDAR cơ khí và IMU cung cấp định hướng; phải đối chiếu chính xác loại cảm biến và nhánh ROS. |
| **FAST-LIO2** (2021/2022) | LIO dùng bộ lọc Kalman lặp; ghép điểm trực tiếp với bản đồ, không cần trích đặc trưng thủ công; dùng cấu trúc `ikd-Tree`. [Bài báo](https://arxiv.org/abs/2107.06829), [repo tác giả](https://github.com/hku-mars/FAST_LIO) | Bài báo và gói chính tập trung odometry + mapping; không nên mặc định có sửa bản đồ toàn cục khi quay vòng. | Ứng viên LIO để đo với LiDAR + IMU sau khi kiểm tra định dạng điểm và đồng bộ thời gian. |
| **Point-LIO** (2023) | Cập nhật trạng thái theo từng điểm LiDAR nhằm xử lý chuyển động mạnh và xuất odometry tần số cao. [Bài báo](https://hub.hku.hk/bitstream/10722/331147/1/content.pdf), [repo tác giả](https://github.com/hku-mars/Point-LIO) | Trọng tâm là odometry, không phải back-end đóng vòng. | Đáng xem khi AMR có rung/chuyển động rất nhanh; lợi ích đó chưa được chứng minh cho robot này. Đồng bộ cảm biến vẫn quan trọng. |
| **KISS-ICP** (2023) | Odometry chỉ dùng LiDAR với ICP điểm–điểm và ít tham số. [Bài báo/repo tác giả](https://github.com/PRBonn/kiss-icp) | Không; đây là pipeline **odometry**, không phải hệ SLAM toàn cục. | Baseline đơn giản để so với LIO và phát hiện phần lợi ích thực sự của IMU. Repo hiện ưu tiên ROS 2. |
| **Cartographer 3D** | SLAM theo *submap*, ghép scan cục bộ và tối ưu *pose graph* toàn cục; 3D dùng LiDAR + IMU. [Tài liệu dự án](https://github.com/cartographer-project/cartographer_ros/blob/master/docs/source/algo_walkthrough.rst), [ROS API](https://github.com/cartographer-project/cartographer_ros/blob/master/docs/source/ros_api.rst) | Có ràng buộc đóng vòng và tối ưu toàn cục. | Ứng viên khi cần bản đồ nhất quán trên vòng đi dài; cần đánh giá cấu hình và khả năng chạy trên ROS đã chọn. |
| **RTAB-Map** | Khung SLAM có phát hiện nơi đã đi qua, tối ưu đồ thị và nhiều kiểu đầu vào, gồm LiDAR 3D trong ROS. [Core](https://github.com/introlab/rtabmap), [ROS wrapper](https://github.com/introlab/rtabmap_ros) | Có. | Linh hoạt nhưng nhiều thành phần/cấu hình; cần chọn rõ nguồn odometry và kiểu bản đồ. |

**Cách đọc bảng:** “có đóng vòng” chỉ mô tả khả năng của phương pháp/gói, không khẳng định cấu hình mặc định đã bật, chạy chính xác hay phù hợp thời gian thực trên máy của ta. Các bài báo đánh giá trên cảm biến, chuyển động và dữ liệu riêng; không có xếp hạng “tốt nhất” chung. Tình trạng ROS/nhánh phần mềm phải kiểm tra lại lúc tích hợp.

## Đề xuất chọn baseline cho AMR hiện tại

Đây là **suy luận của Agent 2**, chưa là quyết định kỹ thuật:

1. **Đầu tiên, chạy KISS-ICP** trên một bag LiDAR để có mốc odometry đơn giản; cùng lúc kiểm tra timestamp, frame, nhiễu và hình dạng đám mây điểm. Nó không kiểm tra lợi ích IMU và không đủ để kết luận về SLAM toàn cục.
2. **Tiếp theo, chạy một LIO thực sự**, ưu tiên thử FAST-LIO2 và LIO-SAM trên cùng bag sau khi biết model LiDAR, trường dữ liệu từng điểm, loại IMU và phiên bản ROS. LIO-SAM phù hợp để học factor graph/đóng vòng; FAST-LIO2 là đối chứng dùng bộ lọc và ghép điểm trực tiếp. Đây là cặp so sánh về thiết kế, không phải khẳng định cặp này sẽ thắng.
3. **Nếu đầu ra phải là bản đồ nhất quán sau nhiều vòng đi**, thử cấu hình đóng vòng của LIO-SAM và/hoặc Cartographer 3D, RTAB-Map. Đo riêng chất lượng odometry trước đóng vòng và bản đồ sau đóng vòng.

Chỉ chọn baseline chính sau khi có: **loại LiDAR, loại và tần số IMU, trường thời gian và `ring` của `PointCloud2`, hiệu chuẩn LiDAR–IMU, ROS 1/2, cấu hình máy, đầu ra mong muốn và bag có đường quay vòng**. Với mỗi lần thử, giữ cùng dữ liệu và hệ tọa độ; ghi cấu hình, phiên bản mã, thời gian chạy, số lần tracking lỗi, độ lệch quỹ đạo khi có ground truth và mức lệch lúc quay về điểm xuất phát. Mốc 0 trong [lộ trình](../ROADMAP.md) chưa chốt các thông tin này.

## Từ vựng tối thiểu

| Thuật ngữ | Hiểu ngắn gọn |
| --- | --- |
| **Pose / tư thế** | Vị trí và hướng của robot trong 3D (6 bậc tự do). |
| **Point cloud** | Tập điểm 3D LiDAR đo được trong một khoảng thời gian. |
| **Scan matching / registration** | Tìm phép tịnh tiến và quay để hai lượt đo/bản đồ trùng nhau. |
| **ICP** | Họ phương pháp ghép hai tập điểm bằng lặp tìm tương ứng rồi cập nhật phép biến đổi. |
| **Deskew / khử méo** | Bù chuyển động trong thời gian LiDAR quét một lượt. |
| **Extrinsic / ngoại chuẩn** | Vị trí và hướng cố định của cảm biến này so với cảm biến/robot khác. |
| **Frame / hệ tọa độ** | Trục tham chiếu của dữ liệu; ví dụ `base_link`, LiDAR, IMU, `odom`, `map`. |
| **Drift / trôi** | Sai số vị trí/hướng tích lũy qua thời gian. |
| **Submap** | Bản đồ nhỏ ghép từ một số lượt đo gần nhau. |
| **Pose graph / factor graph** | Mô hình các tư thế và ràng buộc đo đạc để tối ưu quỹ đạo. |
| **Loop closure / đóng vòng** | Nhận ra nơi đã đi qua và thêm ràng buộc để sửa trôi. |
| **Baseline** | Cách chạy tham chiếu với dữ liệu, cấu hình và phép đo được lưu để so sánh công bằng. |

## Nguồn trực tiếp để đọc tiếp

- Zhang & Singh, **LOAM: Lidar Odometry and Mapping in Real-time**, RSS 2014: [trang bài báo tại CMU](https://publications.ri.cmu.edu/loam-lidar-odometry-and-mapping-in-real-time).
- Shan & Englot, **LeGO-LOAM**, IROS 2018: [repo nhóm tác giả và bản preprint](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM).
- Shan và cộng sự, **LIO-SAM**, IROS 2020: [bài báo](https://arxiv.org/abs/2007.00258), [repo tác giả](https://github.com/TixiaoShan/LIO-SAM).
- Xu và cộng sự, **FAST-LIO2**, IEEE T-RO 2022: [bài báo](https://arxiv.org/abs/2107.06829), [repo tác giả](https://github.com/hku-mars/FAST_LIO).
- He và cộng sự, **Point-LIO**, 2023: [bài báo tại HKU](https://hub.hku.hk/bitstream/10722/331147/1/content.pdf), [repo tác giả](https://github.com/hku-mars/Point-LIO).
- Vizzo và cộng sự, **KISS-ICP**, IEEE RA-L 2023: [repo tác giả và liên kết bài báo](https://github.com/PRBonn/kiss-icp).
- **Cartographer ROS**, [algorithm walkthrough](https://github.com/cartographer-project/cartographer_ros/blob/master/docs/source/algo_walkthrough.rst) và [ROS API](https://github.com/cartographer-project/cartographer_ros/blob/master/docs/source/ros_api.rst), do dự án duy trì.
- **RTAB-Map**, [thư viện gốc](https://github.com/introlab/rtabmap) và [ROS wrapper](https://github.com/introlab/rtabmap_ros), do dự án duy trì.
