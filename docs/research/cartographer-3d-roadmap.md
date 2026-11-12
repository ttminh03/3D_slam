# Roadmap triển khai và học sâu Cartographer 3D

> Tài liệu nghiên cứu cho AMR LiDAR 3D + IMU. Cartographer 3D là hướng triển khai đầu tiên được chọn để học và thử nghiệm, chưa phải kết luận về chất lượng trên robot này. Mọi ô dưới đây bắt đầu ở trạng thái chưa hoàn thành.

## Mục tiêu và cách làm

Đích kỹ thuật là một hệ thống nhận LiDAR 3D + IMU, ước lượng quỹ đạo 6 bậc tự do, tạo submap 3D, nhận diện vòng lặp, tối ưu pose graph và xuất quỹ đạo/bản đồ. Đích học tập là giải thích và kiểm chứng được từng phép biến đổi, hàm mục tiêu và điều kiện thất bại; kết quả chạy được một gói có sẵn chưa đủ để hoàn thành roadmap.

Đi theo hai đường liên tiếp:

1. **Chạy bản gốc làm mốc tham chiếu:** cố định phiên bản mã, cấu hình, bag và số đo. Bản gốc giúp kiểm tra hợp đồng dữ liệu và cung cấp đầu ra để đối chiếu.
2. **Tự triển khai từng khối trong repo:** viết lõi thuật toán có giao diện dữ liệu rõ ràng, sau đó nối ROS; mỗi khối có ví dụ hình học nhỏ và thí nghiệm trên cùng bag. Không đặt mục tiêu tương thích định dạng `.pbstream` của Cartographer nếu chưa có quyết định riêng.

Mỗi mốc chỉ coi là xong khi có bốn hiện vật: (a) ghi chú giải thích bằng lời và công thức, (b) đường đi dữ liệu qua hàm gốc, (c) mã hoặc cấu hình chạy được, (d) kết quả kiểm chứng và trường hợp thất bại. Ghi commit, bag/config, seed nếu có, máy chạy và đơn vị đo theo [quy trình](../WORKFLOW.md).

## Những gì đã xác minh từ nguồn gốc

- Bài báo Hess và cộng sự, *Real-Time Loop Closure in 2D LIDAR SLAM* (ICRA 2016), giải thích tư duy submap, ràng buộc và branch-and-bound **cho 2D**. [Tài liệu thuật toán của Cartographer ROS](https://github.com/cartographer-project/cartographer_ros/blob/master/docs/source/algo_walkthrough.rst) nói các khái niệm thường áp dụng cho 3D; chi tiết 3D trong roadmap này được đối chiếu thêm với mã nguồn 3D, không coi bài báo 2D là chứng minh đầy đủ cho bản 3D. [Bài báo gốc](https://research.google/pubs/real-time-loop-closure-in-2d-lidar-slam/)
- `LocalTrajectoryBuilder3D` nhận IMU trước khi xử lý range data, gom dữ liệu, ước lượng tư thế tại thời điểm điểm đo, lọc voxel, ghép scan với submap rồi chèn kết quả vào submap. Bước ghép dùng hai độ phân giải; bộ ghép tương quan trực tuyến là tùy chọn, sau đó là tối ưu Ceres. [Mã nguồn local trajectory builder](https://github.com/cartographer-project/cartographer/blob/master/cartographer/mapping/internal/3d/local_trajectory_builder_3d.cc)
- `Submap3D` giữ lưới voxel xác suất ở hai độ phân giải. `ActiveSubmaps3D` duy trì tối đa hai submap đang chồng lấn. Bộ chèn cập nhật ô trúng tia và một số ô trống gần điểm trúng theo cấu hình. [Submap 3D](https://github.com/cartographer-project/cartographer/blob/master/cartographer/mapping/3d/submap_3d.cc), [range inserter 3D](https://github.com/cartographer-project/cartographer/blob/master/cartographer/mapping/3d/range_data_inserter_3d.cc)
- Back-end tìm ràng buộc node–submap bằng fast correlative scan matcher 3D, loại theo điểm số rồi tinh chỉnh bằng Ceres; `PoseGraph3D` chuyển các ràng buộc vào bài toán tối ưu toàn cục. [Constraint builder 3D](https://github.com/cartographer-project/cartographer/blob/master/cartographer/mapping/internal/constraints/constraint_builder_3d.cc), [pose graph 3D](https://github.com/cartographer-project/cartographer/blob/master/cartographer/mapping/internal/3d/pose_graph_3d.cc), [optimization problem 3D](https://github.com/cartographer-project/cartographer/blob/master/cartographer/mapping/internal/optimization/optimization_problem_3d.cc)
- Trong nhánh ROS 2 đã đọc, `SensorBridge::ToImuData` yêu cầu IMU và `tracking_frame` **cùng vị trí**. `msg_conversion.cpp` gán thời gian mỗi điểm bằng `0` khi `PointCloud2` thiếu trường `time`. Đây là hành vi của nhánh đã đọc; phải đối chiếu lại bản cài đặt được chọn trước khi tích hợp. [ROS 2 sensor bridge](https://github.com/ros2/cartographer_ros/blob/ros2/cartographer_ros/src/sensor_bridge.cpp), [ROS 2 message conversion](https://github.com/ros2/cartographer_ros/blob/ros2/cartographer_ros/src/msg_conversion.cpp)

## Bản đồ đọc mã nguồn theo thứ tự

| Chặng đọc | Mã nguồn gốc và câu hỏi phải trả lời |
| --- | --- |
| Cổng ROS | [ROS 2 `node.cpp`](https://github.com/ros2/cartographer_ros/blob/ros2/cartographer_ros/src/node.cpp), [`sensor_bridge.cpp`](https://github.com/ros2/cartographer_ros/blob/ros2/cartographer_ros/src/sensor_bridge.cpp), [`msg_conversion.cpp`](https://github.com/ros2/cartographer_ros/blob/ros2/cartographer_ros/src/msg_conversion.cpp): topic nào được đăng ký, timestamp và TF được đổi thế nào, dữ liệu nào bị bỏ? |
| Dự đoán chuyển động | [`imu_tracker.cc`](https://github.com/cartographer-project/cartographer/blob/master/cartographer/mapping/imu_tracker.cc), [`pose_extrapolator.cc`](https://github.com/cartographer-project/cartographer/blob/master/cartographer/mapping/pose_extrapolator.cc), [`local_trajectory_builder_3d.cc`](https://github.com/cartographer-project/cartographer/blob/master/cartographer/mapping/internal/3d/local_trajectory_builder_3d.cc): IMU giúp ước lượng trọng lực, hướng và tư thế từng điểm thế nào? |
| Local SLAM | [`local_trajectory_builder_3d.cc`](https://github.com/cartographer-project/cartographer/blob/master/cartographer/mapping/internal/3d/local_trajectory_builder_3d.cc), [`ceres_scan_matcher_3d.cc`](https://github.com/cartographer-project/cartographer/blob/master/cartographer/mapping/internal/3d/scan_matching/ceres_scan_matcher_3d.cc), [`occupied_space_cost_function_3d.h`](https://github.com/cartographer-project/cartographer/blob/master/cartographer/mapping/internal/3d/scan_matching/occupied_space_cost_function_3d.h): tư thế dự đoán, mục tiêu occupied-space và prior được ghép ra sao? |
| Bản đồ cục bộ | [`submap_3d.cc`](https://github.com/cartographer-project/cartographer/blob/master/cartographer/mapping/3d/submap_3d.cc), [`range_data_inserter_3d.cc`](https://github.com/cartographer-project/cartographer/blob/master/cartographer/mapping/3d/range_data_inserter_3d.cc): vì sao có hai độ phân giải và hai submap hoạt động? |
| Đóng vòng | [`rotational_scan_matcher.cc`](https://github.com/cartographer-project/cartographer/blob/master/cartographer/mapping/internal/3d/scan_matching/rotational_scan_matcher.cc), [`fast_correlative_scan_matcher_3d.cc`](https://github.com/cartographer-project/cartographer/blob/master/cartographer/mapping/internal/3d/scan_matching/fast_correlative_scan_matcher_3d.cc), [`constraint_builder_3d.cc`](https://github.com/cartographer-project/cartographer/blob/master/cartographer/mapping/internal/constraints/constraint_builder_3d.cc): từ ứng viên đến ràng buộc được chấp nhận qua những ngưỡng nào? |
| Tối ưu toàn cục | [`pose_graph_3d.cc`](https://github.com/cartographer-project/cartographer/blob/master/cartographer/mapping/internal/3d/pose_graph_3d.cc), [`optimization_problem_3d.cc`](https://github.com/cartographer-project/cartographer/blob/master/cartographer/mapping/internal/optimization/optimization_problem_3d.cc), [`spa_cost_function_3d.h`](https://github.com/cartographer-project/cartographer/blob/master/cartographer/mapping/internal/optimization/cost_functions/spa_cost_function_3d.h): biến tối ưu, residual, trọng số và cập nhật quỹ đạo là gì? |

Trước khi bắt đầu triển khai, lưu commit SHA chính xác của hai repo upstream. Các liên kết `master`/`ros2` ở trên phục vụ đọc và có thể thay đổi theo thời gian. Mẫu `backpack_3d.lua` của [nhánh ROS 2](https://github.com/ros2/cartographer_ros/blob/ros2/cartographer_ros/configuration_files/backpack_3d.lua) là điểm tham khảo cấu trúc, không phải cấu hình có thể chép nguyên cho AMR.

### Bốn phép tính phải tự giải thích được

1. **Đổi frame tại đúng thời điểm:** `p_map(t) = T_map_tracking(t) · T_tracking_lidar · p_lidar`. Giải thích chiều của từng phép biến đổi và vì sao dùng `t` của điểm đo khi dữ liệu có thời gian từng điểm.
2. **Cập nhật voxel:** một hit làm tăng xác suất chiếm chỗ, còn phần tia được xem là trống làm giảm xác suất; biểu diễn bằng odds/log-odds và chỉ ra cách tránh cập nhật lặp một ô trong một lần chèn. Đối chiếu với [range inserter 3D](https://github.com/cartographer-project/cartographer/blob/master/cartographer/mapping/3d/range_data_inserter_3d.cc).
3. **Scan matching:** với mỗi điểm, residual occupied-space tỷ lệ với `1 − P(T·p)`; cộng prior tịnh tiến và quay quanh tư thế dự đoán. Tự giải thích hệ số `1/√N`, nội suy xác suất và trường hợp hình học không đủ ràng buộc. Đối chiếu với [cost function](https://github.com/cartographer-project/cartographer/blob/master/cartographer/mapping/internal/3d/scan_matching/occupied_space_cost_function_3d.h) và [Ceres matcher](https://github.com/cartographer-project/cartographer/blob/master/cartographer/mapping/internal/3d/scan_matching/ceres_scan_matcher_3d.cc).
4. **Pose graph:** một ràng buộc đo chuyển động tương đối giữa node và submap; so `T_i⁻¹·T_j` với phép đo rồi cân residual tịnh tiến/quay. Tự chỉ ra gauge freedom, tác động của loop closure sai và cách IMU đi vào tối ưu 3D. Đối chiếu với [SPA cost](https://github.com/cartographer-project/cartographer/blob/master/cartographer/mapping/internal/optimization/cost_functions/spa_cost_function_3d.h) và [bài toán tối ưu](https://github.com/cartographer-project/cartographer/blob/master/cartographer/mapping/internal/optimization/optimization_problem_3d.cc).

## Các mốc thực hiện

### M0 — Chốt hợp đồng bài toán và dữ liệu

- [ ] Ghi đầu ra cần thiết: quỹ đạo, submap/bản đồ 3D, định dạng xuất; yêu cầu thời gian thực và giới hạn CPU/RAM. Chốt phiên bản ROS và máy thử bằng quyết định kỹ thuật; ROS 2 Humble + Gazebo Classic trong repo hiện chỉ là **cấu hình chạy thử**.
- [ ] Đo thực tế `/points`, `/imu/data`, `/tf`, `/tf_static`, `/clock`: tần số, `frame_id`, tính tăng dần của timestamp, trường `PointCloud2` (`x/y/z`, `time` nếu có), gia tốc và tốc độ góc IMU. Kiểm tra trọng lực chưa bị loại khỏi gia tốc. [Hướng dẫn dữ liệu Cartographer](https://github.com/cartographer-project/cartographer_ros/blob/master/docs/source/your_bag.rst)
- [ ] Kiểm tra phép biến đổi cố định `base_link→lidar_link`, `base_link→imu_link`; chọn `tracking_frame` tại `imu_link` cho bản tham chiếu ROS 2 để thỏa điều kiện cùng vị trí IMU. Ghi rõ nút nào phát mỗi cạnh TF động để không có hai nguồn cùng phát `odom→base_footprint`.
- [ ] Ghi ít nhất một bag ngắn có đoạn thẳng, quay tại chỗ và quay về điểm đã đi qua; ghi cấu hình cảm biến và checksum/phiên bản dữ liệu ở ngoài Git. Dữ liệu mẫu hiện cấu hình LiDAR 360×16 tia ở 20 Hz, IMU 100 Hz, nhưng tần số thực tế và trường điểm chưa được kiểm chứng. [AMR mẫu](../../src/amr_description/README.md)

**Đạt khi:** có bảng hợp đồng dữ liệu, bag phát lại được, TF hợp lệ và quyết định về nền tảng. Nếu thiếu thời gian từng điểm, ghi rõ hạn chế khử méo; không tự tạo `time` giả rồi coi là đo đạc thật.

### M1 — Chạy Cartographer 3D gốc làm mốc tham chiếu

- [ ] Cố định commit hoặc gói nhị phân Cartographer core + ROS wrapper; tạo Lua/launch riêng cho **một** `PointCloud2` và IMU. Ban đầu tắt `use_intensities`; chưa dùng wheel odometry làm đầu vào để nhìn rõ hành vi LiDAR–IMU. Chọn chiến lược TF cùng với cấu hình `provide_odom_frame`, `published_frame` và plugin diff-drive.
- [ ] Chạy lại chính bag M0; ghi startup log, số scan/node/submap, quỹ đạo, constraint, thời gian chạy và bộ nhớ. Chỉ sau khi luồng cơ bản hoạt động mới thay đổi một nhóm tham số mỗi lần.
- [ ] Lưu kết quả tham chiếu cùng script chạy. Nếu có pose thật từ mô phỏng, ghi ở kênh riêng **chỉ để đánh giá**, không cấp cho SLAM; `/odom` bánh xe không tự động là ground truth.

**Đạt khi:** người khác phát lại bag và thấy quỹ đạo cùng submap 3D, không lỗi thời gian/TF. Trường hợp thiếu `time` vẫn có thể thử chạy, nhưng phải ghi rõ mức khử méo đạt được và chưa kết luận về robot chuyển động nhanh.

### M2 — Nền toán học và dữ liệu để tự triển khai

- [ ] Viết ghi chú và ví dụ số về phép biến đổi `SE(3)`, quaternion, đổi điểm giữa `lidar`, `imu`, `base`, `local`, `map`; kiểm chứng phép nghịch đảo và hợp thành trên tư thế đã biết.
- [ ] Dựng lớp đọc bag độc lập với thuật toán: kiểm tra thứ tự timestamp, đồng bộ IMU–LiDAR, ngoại chuẩn, chia scan và lọc điểm. Tạo thí nghiệm nhân tạo có quỹ đạo biết trước để kiểm chứng bù chuyển động; nếu dữ liệu thật thiếu thời gian điểm thì chỉ kiểm chứng theo mức thời gian thông điệp có sẵn.
- [ ] Đọc `pose_extrapolator` rồi triển khai bộ dự đoán tư thế từ IMU và các pose đã ghép. Ghi giả thiết về trọng lực, bias, khởi tạo và độ tin cậy khi AMR đứng yên.

**Đạt khi:** ví dụ số tái tạo đúng phép biến đổi và dữ liệu kiểm thử xác định được sai lệch do cố ý đổi timestamp/ngoại chuẩn. Có sơ đồ đơn vị và frame cho mọi dữ liệu nội bộ.

### M3 — Submap xác suất 3D

- [ ] Tự triển khai lưới voxel xác suất thưa: chỉ số voxel, log-odds hoặc cách cập nhật tương đương, hit/miss, giới hạn xác suất, truy vấn nội suy cho scan matching. Giải thích vì sao cập nhật hit/miss theo tia và vì sao chỉ cập nhật một đoạn free-space theo cấu hình gốc.
- [ ] Dựng hai mức phân giải và hai submap hoạt động có khoảng chồng lấn; kiểm chứng bằng cảnh hình học nhỏ (tường, góc, cột) với pose biết trước.
- [ ] Đo dung lượng bộ nhớ, số voxel, ảnh hưởng độ phân giải và số tia; tách biểu diễn 3D thực khỏi hình chiếu 2D dùng để xem trong RViz.

**Đạt khi:** map đồ chơi có ô chiếm/trống đúng kỳ vọng và submap chuyển trạng thái đúng. Chưa cần loop closure.

### M4 — Front-end 3D và quỹ đạo cục bộ

- [ ] Triển khai tích lũy scan, voxel/adaptive voxel filter, dự đoán pose, ghép scan với submap rồi cập nhật pose và map. Bắt đầu bằng mục tiêu occupied-space + prior vị trí/hướng của Ceres matcher; thêm tìm kiếm tương quan trực tuyến khi thấy cần từ số đo.
- [ ] Dẫn xuất hàm mục tiêu, xác định đầu vào/đầu ra và kiểm tra cực tiểu trên cảnh nhân tạo với phép biến đổi biết trước. Theo dõi residual, điểm bị lọc và số lần scan matching không tạo kết quả.
- [ ] Chạy bag M0 với back-end tắt để đo riêng drift cục bộ, thời gian mỗi scan và tình huống ít cấu trúc; so với mốc M1 bằng cùng frame/dữ liệu, không coi khác biệt nhỏ là chứng minh tương đương thuật toán.

**Đạt khi:** quỹ đạo cục bộ liên tục, submap tăng theo hành trình, có báo cáo sai số và tốc độ trên bag cố định.

### M5 — Đóng vòng và tối ưu pose graph 3D

- [ ] Tạo node, submap và ràng buộc trong submap. Dựng ứng viên đóng vòng trên submap đã hoàn tất; nghiên cứu histogram quay, fast correlative scan matcher 3D, ngưỡng điểm số và tinh chỉnh Ceres. Kiểm tra false positive bằng hai vùng có hình học gần giống nhau.
- [ ] Triển khai tối ưu toàn cục cho tư thế node/submap với residual tương đối và ràng buộc IMU phù hợp; cố định gauge, theo dõi residual trước/sau tối ưu, chặn ràng buộc sai. Đọc `OptimizationProblem3D::Solve` để hiểu những biến và trọng số mà bản gốc thực sự tối ưu.
- [ ] Dùng một vòng lặp nhân tạo trước, sau đó bag M0. Đo độ lệch khi quay về, thay đổi quỹ đạo sau loop closure, số ràng buộc được chấp nhận/sai và thời gian tối ưu.

**Đạt khi:** một vòng lặp đúng làm giảm sai lệch toàn cục mà không phá submap cục bộ; một ràng buộc sai bị loại trong ca thử đã tạo.

### M6 — Tích hợp ROS, xuất kết quả và đánh giá

- [ ] Nối lõi tự triển khai với ROS theo giao diện đã chốt; cấu hình `use_sim_time`, remap, TF và nơi lưu map/quỹ đạo. Chỉ một nút sở hữu mỗi phép biến đổi TF động.
- [ ] Đánh giá quỹ đạo bằng ATE/RPE khi có ground truth độc lập; ghi phép căn chỉnh, frame, đơn vị và thời điểm lấy mẫu. Khi chưa có ground truth, dùng độ lệch quay về và tính nhất quán của các ràng buộc như chỉ báo, ghi rõ giới hạn của chúng. Đo chất lượng bản đồ 3D bằng hình học mô phỏng hoặc mốc độc lập nếu có.
- [ ] Kiểm tra ca thiếu IMU, gián đoạn timestamp, đổi ngoại chuẩn, chuyển động nhanh, hành lang lặp lại và chạy dài; đo CPU/RAM và độ trễ trên cùng máy. Lưu cấu hình và script để chạy lại.

**Đạt khi:** người khác có thể chạy đầu cuối, đọc kết quả và tái lập phép đo; giới hạn và lỗi đã biết được ghi rõ.

## Trạng thái thử ngày 2026-11-12

- **M0 đã kiểm tra một phần:** mô phỏng headless chạy được. Bag `/tmp/carto3d_amr_2026-11-12` chứa 557 cloud ở 20 Hz và 2785 mẫu IMU gần 100 Hz theo thời gian mô phỏng; timestamp tăng đều, IMU có trọng lực khi đứng yên và TF nối được các frame cảm biến. Cloud thiếu trường `time` từng điểm và `ring`. Bag nằm trong `/tmp`, chưa là dữ liệu lưu bền hoặc bộ đánh giá đã chốt.
- **M1 đã chạy smoke test:** Cartographer 3D gốc tạo trajectory ID 0 với 43 pose, một submap 3D đang hoạt động, TF `map→odom` và file `.pbstream`. Chưa có submap hoàn tất hoặc ràng buộc đóng vòng; chưa đo ATE/RPE. Phiên bản gói, checksum bag, lệnh chạy và log được ghi trong [README của package thử](../../src/amr_cartographer/README.md). Đây là kiểm chứng tích hợp, chưa là baseline chất lượng.
- Các ô M0/M1 ở trên vẫn mở vì chưa đáp ứng đủ điều kiện hoàn thành của từng mốc.

## Việc kế tiếp

Ghi một bag dài hơn có đường khép vòng đã mô tả bằng lệnh/route tái lập được; lưu dữ liệu ngoài `/tmp` theo quy tắc quản lý dữ liệu của dự án. Trước khi kết luận về đóng vòng, cần có submap hoàn tất và kiểm tra ràng buộc ngoài submap. Song song, hoàn tất các quyết định M0 về đầu ra, phiên bản ROS, phần cứng và chỉ số chất lượng. Không dùng smoke test này để kết luận về thời gian thực hoặc khử méo trong chuyển động nhanh.

## Tồn đọng cần theo dõi

- **critical:** hiện chưa có trở ngại chặn chạy thử tích hợp tiếp theo.
- **warn:** bag hiện ở `/tmp` và route chưa tự động hóa; cloud thiếu thời gian từng điểm; bag ngắn chưa tạo submap hoàn tất/đóng vòng. Cần pin phiên bản/commit nguồn khi chuyển sang tự triển khai.
- **info:** mô phỏng còn dùng cảm biến/kích thước mẫu; đầu ra cuối, ROS distro, phần cứng và ngưỡng chất lượng chưa được chốt theo Mốc 0 chung.
