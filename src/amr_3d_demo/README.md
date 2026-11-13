# Demo LiDAR odometry 3D trên mô phỏng AMR

Đây là một lát cắt nhỏ để học cách biến scan 3D thành pose và local map. **Chưa phải LIO-SAM**: chưa có feature edge/plane, IMU preintegration, factor graph, loop closure hay deskew tịnh tiến.

## Giả thuyết và đầu vào

- Gazebo xuất `/points` (`sensor_msgs/PointCloud2`) trong `lidar_link` với `x,y,z` kiểu `float32` (có thể thêm `intensity`). Trường `time` kiểu `float32` hoặc `float64`, nếu có, là **số giây từ `header.stamp`** của từng điểm. Không dùng `ring` trong ICP.
- `/imu/data` (`sensor_msgs/Imu`) có angular velocity và `header.frame_id=imu_link`. TF tĩnh `lidar_link <- imu_link` dùng để đổi trục gyro. Đang giả định hai cảm biến gắn cứng; deskew chỉ bù quay, bỏ qua tịnh tiến và lever arm.
- Mọi timestamp thuộc cùng đồng hồ `/clock`; node dùng `use_sim_time=true`. Dữ liệu IMU phải phủ cả scan mới có thể deskew. Nếu cloud thiếu `time` hoặc IMU chưa phủ đủ, cloud đi qua nguyên trạng và `/demo/status` báo lý do. Không tạo thời gian điểm từ thứ tự điểm.
- Subscription dùng QoS cảm biến (`best effort`); hàng đợi cloud chỉ giữ scan mới nhất khi xử lý chậm, còn hàng đợi IMU sâu hơn để phục vụ tích phân gyro. Node đợi tối đa 180 ms thời gian thực cho IMU phủ timestamp của scan (và điểm cuối nếu có `time`); hết hạn thì xử lý với fallback được ghi trong `/demo/status`.
- `/odom` **không phải đầu vào** của demo. Nó là wheel odometry trong mô phỏng, có thể dùng đối chiếu thô sau khi căn hệ tọa độ ban đầu.

## Chạy

Từ thư mục gốc workspace:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select amr_description amr_3d_demo
source install/setup.bash
ros2 launch amr_description sim.launch.py gui:=false
```

Trong terminal khác:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch amr_3d_demo demo.launch.py
```

Trong terminal thứ ba, đọc trạng thái:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic echo /demo/status
```

Sau khi cả hai launch đã chạy, mở terminal thứ tư để cho robot di chuyển bằng `/cmd_vel`:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic pub -r 5 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.15}, angular: {z: 0.10}}'
```

Ctrl+C để dừng lệnh điều khiển. Mở RViz **sau khi `/points` đã có dữ liệu**, đặt **Global Options → Fixed Frame = `demo_map`** và **Views → Target Frame = `demo_map`** để camera đứng yên trong hệ bản đồ. Thêm `Odometry` `/demo/odom`, `Path` `/demo/path` và `PointCloud2` **`/demo/map`**; tắt display `LaserScan` `/scan` cũ nếu nó còn trong cấu hình RViz. Node phát TF `demo_map → demo_lidar_pose` từ scan đầu; child `demo_lidar_pose` là pose ước lượng tại vị trí LiDAR, tách khỏi TF thật `lidar_link` của robot để tránh xung đột. `/demo/map` phát khi thêm keyframe và phát lại mỗi giây để RViz mở muộn vẫn thấy bản đồ. Publisher còn dùng Transient Local; nếu muốn nhận ngay bản tin cuối khi vừa mở RViz, đặt Durability của display PointCloud2 thành `Transient Local`. `/demo/local_map` là cửa sổ scan matching gồm 8 keyframe gần nhất, nên nội dung nó thay đổi khi robot đi xa hoặc quay đủ lâu; chỉ thêm display đó khi muốn xem target ICP.

## Thuật toán và giao diện

1. Kiểm tra `frame_id`, layout PointCloud2, timestamp, điểm hữu hạn; loại điểm ngoài `[0.3, max_range]` mét.
2. Nếu có `time` và IMU phủ đủ, tích phân gyro theo thời gian và đổi mỗi điểm về frame LiDAR tại `header.stamp`: `p_ref = R_refᵀ R_point p_point`. Khi thiếu `time`, log một lần và bỏ qua deskew.
3. Lọc voxel bằng trung bình tọa độ trong mỗi ô, rồi lấy mẫu trải đều trên toàn cloud đã lọc khi vượt `max_scan_points` (mặc định 500) để giới hạn chi phí ICP. Scan đầu tiên định nghĩa `demo_map` và tạo keyframe đầu.
4. Với scan tiếp theo, tích phân gyro giữa hai timestamp scan để dự đoán hướng ban đầu cho ICP; nếu IMU không phủ khoảng đó, dùng pose trước (`seed=last_pose` trong status). Tìm điểm gần nhất trong local map bằng KD-tree. Residual 3D `r_i = T p_i - q_i`; Jacobian cho bước cập nhật trái là `J_i = [-[T p_i]×, I]`. Tính `H = Σ w_i J_iᵀJ_i`, `g = Σ w_i J_iᵀr_i`, giải `H δ = -g`, rồi cập nhật pose. Trọng số Huber hạn chế tác động của cặp điểm xa. Nếu cặp điểm không đủ, hình học suy biến hoặc bước cập nhật quá lớn, giữ pose cũ và báo trạng thái lỗi.
5. Khi pose đi đủ xa/quay đủ góc, đưa scan vào keyframe. Local map lấy từ tối đa 8 keyframe gần đây và lọc voxel làm target ICP. Đồng thời tích lũy điểm keyframe trong `demo_map` thành `/demo/map` để hiển thị; map này có voxel 0.35 m và giới hạn 30.000 ô theo mặc định. Khi đầy, ô cũ vẫn cập nhật nhưng ô mới bị bỏ và node cảnh báo. Đây là **point cloud tích lũy**, không phải occupancy map hoặc pose graph.

Các topic xuất: `/demo/odom` (`nav_msgs/Odometry`, pose ước lượng của LiDAR trong `demo_map`, `child_frame_id=demo_lidar_pose`), `/demo/path` (`nav_msgs/Path`), `/demo/map` (point cloud tích lũy), `/demo/local_map` (cửa sổ ICP), `/demo/status` (`std_msgs/String`) và TF động `demo_map → demo_lidar_pose`. Tất cả cloud map có `header.frame_id=demo_map`; TF có timestamp của scan và tiếp tục phát pose giữ nguyên khi tracking thất bại. `demo_map` bắt đầu tại pose LiDAR của scan đầu, nên không cùng gốc với `odom` Gazebo. Nếu tracking thất bại, `/demo/odom` giữ pose trước và path không thêm điểm mới. Các số trong `odom.pose.covariance` (0.1 khi khớp, 1000 khi thất bại) chỉ là **giá trị hiển thị để đánh dấu trạng thái**, không phải hiệp phương sai tính từ ICP. `odom.twist` chưa được ước lượng; các giá trị số 0 không biểu thị vận tốc thật và phương sai twist được đặt lớn. `/demo/status` có `scan_points`, `map_points` (local), `global_map_points`, `wait_ms` (chờ IMU sau khi nhận cloud), `processing_ms` (xử lý sau khi hết chờ) và `latency_ms` (tổng hai số đó) đo theo đồng hồ steady của máy.

## Kiểm chứng tối thiểu

```bash
source /opt/ros/humble/setup.bash
colcon test --packages-select amr_3d_demo --event-handlers console_direct+
colcon test-result --verbose
ros2 topic hz /points
ros2 topic hz /demo/odom
```

Kiểm tra `/demo/status`: ban đầu `initialized`, sau đó `matched`; với mô phỏng hiện tại, `deskew=off_missing_point_time` là đúng. Các bài test tạo chuyển động quay có ground truth để xác nhận deskew, tạo biến đổi 3D biết trước để kiểm tra ICP, và tạo hình học đường thẳng suy biến để xác nhận từ chối pose. Thử robot đứng yên, đi thẳng rồi quay; ghi số lần tracking thất bại, sai lệch quỹ đạo sau khi căn gốc và thời gian xử lý scan trên cùng cấu hình máy. Chưa có ATE/RPE chuẩn vì `/odom` wheel không phải ground truth tuyệt đối.

### Kết quả thử ban đầu, trước khi sửa map khi quay (2026-11-13)

Trên Intel Core i7-10750H, ROS 2 Humble, Gazebo headless với `demo.world` và cấu hình mặc định `max_scan_points=500`:

| Thử nghiệm | Scan báo `matched` | `processing_ms` trung vị | `processing_ms` lớn nhất |
| --- | ---: | ---: | ---: |
| Robot đứng yên | 119/119 | 39,30 ms | 52,96 ms |
| Robot đi thẳng rồi quay, khoảng 12 giây thời gian thực | 102/102 | 45,14 ms | 284,76 ms |

Trong lượt robot di chuyển, `/demo/odom` đổi từ gần gốc tới `x≈0,622 m`, `y≈0,074 m`. Đây là hai lượt thử ngắn; `matched` chỉ nói ICP tìm được nghiệm theo điều kiện nội bộ, chưa chứng minh pose chính xác. Giá trị lớn nhất 284,76 ms cho thấy vẫn có scan xử lý chậm hơn chu kỳ LiDAR; cần đo nhiều lượt và so quỹ đạo với ground truth trước khi đánh giá độ ổn định, độ chính xác hay khả năng chạy thời gian thực.

### Sau khi sửa TF, chờ IMU và thêm map tích lũy (2026-11-13)

Thử trên cùng máy Intel Core i7-10750H với Gazebo headless và giới hạn 500 điểm scan:

| Thử nghiệm | Kết quả |
| --- | --- |
| Quay tại chỗ | 127/127 scan `matched`; góc quay wheel odom 2,0666 rad, demo 2,0722 rad; sai khác độ dịch LiDAR sau khi tính offset cảm biến khoảng 0,088 m |
| Map tích lũy sau lượt quay | Khoảng cách điểm gần nhất giữa map đầu và cuối: trung vị 0,024 m, phân vị 90% 0,108 m |
| Đi thẳng rồi quay | 155/155 scan `matched`; góc quay wheel odom 0,563 rad, demo 0,559 rad; sai khác độ dịch LiDAR sau khi tính offset khoảng 0,097 m |
| Độ trễ lượt đi thẳng rồi quay | Trung vị chờ IMU 0,56 ms; trung vị tổng độ trễ 73,63 ms/scan |

Wheel odom chỉ là mốc đối chiếu của mô phỏng, chưa phải ground truth độc lập. Vì `/demo/map` giữ các ô đã thấy, chỉ số điểm gần nhất giữa map đầu và cuối chủ yếu kiểm tra **điểm cũ không bị xóa**; nó không đo độ chính xác đăng ký scan mới. Các lượt thử này còn ngắn; độ trễ trung vị 73,63 ms chưa đủ để khẳng định demo xử lý kịp luồng LiDAR 20 Hz trong mọi điều kiện.

## Giới hạn

- ICP điểm–điểm có thể lệch hoặc mắc cực tiểu cục bộ ở tường dài/hành lang đối xứng; demo chưa dùng dự đoán IMU cho pose tịnh tiến và chưa có phục hồi tracking.
- `/demo/local_map` trượt theo keyframe; `/demo/map` giữ ô voxel đã thấy nhưng có cap bộ nhớ. Chúng không có loop closure. Sai pose keyframe vẫn có thể làm tường dày/lệch trong `/demo/map`.
- Deskew quay chỉ có ích trên cloud có thời gian từng điểm đáng tin và IMU/TF/thời gian đồng bộ. Scan mô phỏng hiện thiếu trường này, nên không chứng minh được hiệu quả deskew bằng chính scan đó.
- `rmse` log là sai số của tương ứng hiện tại trong vòng ICP, không phải chỉ số độ chính xác quỹ đạo. Cần ground truth và căn chỉnh quỹ đạo để đo ATE/RPE.
