# Bước 2 — đề cương nghiên cứu sâu LIO-SAM để triển khai lại

> **Trạng thái:** dàn ý học và triển khai, chưa phải bản thiết kế đã chốt. Đọc sau [giáo trình nhập môn](lio-sam-nhap-mon.html). Mỗi chặng dưới đây phải được mở rộng thành bài học kỹ thuật, bài thực hành và một thay đổi mã có thể kiểm chứng độc lập. Thứ tự ưu tiên là **phụ thuộc kiến thức và dữ liệu**, không phải thứ tự file trong repository.

## 1. Đích và cách dùng đề cương

Đích học: tự suy ra công thức, giải thích từng dữ liệu đi qua hệ thống, chỉ ra giả định và trường hợp hỏng. Đích kỹ thuật: triển khai được một đường LiDAR–IMU 3D từ đầu vào hợp lệ đến quỹ đạo, keyframe và bản đồ, rồi thêm hiệu chỉnh dài hạn. Độ ổn định, độ chính xác và chi phí máy được đo riêng; không kết luận một thiết kế tốt chỉ vì RViz cho bản đồ trông hợp lý.

Mỗi bài triển khai về sau cần giữ cùng một cấu trúc:

1. **Câu hỏi cần trả lời:** vấn đề nào buộc module tồn tại; đầu vào/đầu ra và đơn vị.
2. **Kiến thức nền:** ký hiệu, hình học, xác suất hoặc cấu trúc dữ liệu cần học trước.
3. **Từ bài báo đến source:** công thức và ý tưởng của bài báo; hàm/cấu hình thực sự làm gì trong bản mã gốc. Ghi rõ khi hai mô tả khác nhau.
4. **Tự tính một ví dụ nhỏ:** số liệu đủ đơn giản để kiểm tra chiều biến đổi, thời gian, dấu và đơn vị.
5. **Hợp đồng module:** kiểu dữ liệu, frame, timestamp, điều kiện đầu vào, đầu ra khi thiếu dữ liệu, phần nào được phép cấu hình.
6. **Bài code nhỏ và kiểm chứng:** kiểm tra bằng dữ liệu tổng hợp trước; sau đó so với source gốc trên cùng dữ liệu, không chỉ bằng hình ảnh.
7. **Giới hạn:** trường hợp suy biến, độ nhạy tham số, bộ nhớ và thời gian chạy; câu hỏi còn mở phải được ghi lại.

Một chặng chỉ được coi là **đã hiểu để code** khi giải thích được vì sao đầu ra đúng, có ví dụ phản chứng, và chỉ được vị trí tương ứng trong [bài báo gốc](https://arxiv.org/html/2007.00258) cùng [source gốc tại commit 0be1fbe](https://github.com/TixiaoShan/LIO-SAM/blob/0be1fbe6275fb8366d5b800af4fc8c76a885c869/README.md). “Code chạy” và “hiểu thuật toán” là hai điều kiện cần kiểm tra riêng.

### Luồng phụ thuộc

```text
Ký hiệu + hình học + thời gian
          ↓
Hợp đồng LiDAR/IMU + ngoại chuẩn
          ↓
IMU trong một scan → deskew → ảnh khoảng cách → điểm cạnh/mặt
                                                ↓
                                  keyframe cũ → bản đồ cục bộ
                                                ↓
                             tương ứng hình học → tối ưu pose LiDAR
                                                ↓
                     ràng buộc odometry giữa các keyframe

Song song sau khi đã hiểu IMU cơ bản:
IMU tích phân trước + bias + pose LiDAR hiệu chỉnh → đồ thị IMU nhịp nhanh
                              ↕
               đồ thị pose keyframe dài hạn → loop/GPS → map toàn cục
```

**Mốc tích hợp quan trọng:** có thể dựng đường LiDAR scan-to-map với pose khởi tạo đã biết trước khi hoàn thành đồ thị IMU. Đây là bài tập cô lập module, **chưa phải LIO-SAM đầy đủ**. Đường LIO-SAM hoàn chỉnh chỉ có khi pose LiDAR hiệu chỉnh quay lại cập nhật trạng thái/bias IMU. [Bài báo, III-A đến III-C](https://arxiv.org/html/2007.00258); [README, System architecture](https://github.com/TixiaoShan/LIO-SAM/blob/0be1fbe6275fb8366d5b800af4fc8c76a885c869/README.md).

### Quy ước ký hiệu cho các công thức trong dàn ý

- $W$: frame thế giới hoặc bản đồ.
- $B$: frame thân robot; bài báo giả định frame IMU trùng với $B$.
- $L$: frame LiDAR.
- $i,j$: chỉ số thời điểm hoặc trạng thái/keyframe, tùy bài. $k$: chỉ số điểm LiDAR hoặc mẫu IMU; từng công thức sẽ nói rõ.
- $p_A$: tọa độ của điểm $p$ trong frame $A$. Khi nhân với ma trận $4\times4$, viết điểm ở dạng thuần nhất $[x,y,z,1]^\top$.
- $T_{AB}$: phép biến đổi tọa độ **từ frame B sang frame A**; $p_A=T_{AB}p_B$. Đảo chiều bằng $T_{BA}=T_{AB}^{-1}$.
- $T_{WB}\in SE(3)$: pose của body trong world, gồm phép quay $R_{WB}\in SO(3)$ và tịnh tiến $p_{WB}$. Trong ma trận thuần nhất: $T_{WB}=\begin{bmatrix}R_{WB}&p_{WB}\\0&1\end{bmatrix}$.
- $v_W$: vận tốc của body biểu diễn trong frame $W$.
- $b_g$, $b_a$: bias của gyroscope và accelerometer.
- $\hat\omega$, $\hat a$: số đo tốc độ góc và gia tốc từ IMU. Dấu mũ trên pose, ví dụ $\hat T$, chỉ **ước lượng**; ý nghĩa dấu mũ phải đọc theo biến của từng công thức.
- $t_0$: thời điểm tham chiếu của scan. $\tau_k$: thời gian tương đối khi đo điểm $k$; $t_k=t_0+\tau_k$.
- $\operatorname{Exp}$, $\operatorname{Log}$: phép chuyển giữa lượng quay/pose và biểu diễn sai khác nhỏ của chúng trên $SO(3)$ hoặc $SE(3)$.
- $r$: residual, tức sai lệch giữa phép đo và dự đoán theo quy ước dấu đã chọn.
- $\Sigma$: ma trận hiệp phương sai của residual; $\|r\|_\Sigma^2=r^\top\Sigma^{-1}r$ là sai lệch có xét độ tin cậy.
- **[Paper]**: công thức trong bài báo hoặc phép viết lại tương đương.
- **[Source]**: phép tính của source gốc tại commit được dẫn.
- **[Suy ra]**: công thức học tập hoặc kiểm thử suy từ các định nghĩa; chưa khẳng định trùng cách source triển khai.

## 2. Chặng 0 — ý tưởng thiết kế trước khi đọc công thức

### 0.1. Vì sao cần một hệ LiDAR–IMU như LIO-SAM?

- **Bài toán xuất phát:** scan matching LiDAR đơn thuần có thể trôi hoặc thất bại khi robot quay nhanh, scan bị méo và hình học thiếu ràng buộc; IMU cho chuyển động ngắn hạn nhanh nhưng tự nó tích lũy sai số. Bản đồ lớn cũng làm tìm tương ứng đắt hơn. Đọc [bài báo, I–II](https://arxiv.org/html/2007.00258) để biết tác giả đặt vấn đề và giới hạn của các hướng trước đó; ghi ý nào là lập luận của tác giả, ý nào đã được thí nghiệm xác nhận.
- **Ý tưởng trung tâm cần tự kể lại:** dùng chuyển động IMU cho deskew và pose khởi tạo; khớp đặc trưng LiDAR với map cục bộ để hiệu chỉnh pose; dùng pose LiDAR sửa state/bias IMU; chỉ lưu một số scan làm keyframe; thêm loop/GPS khi có bằng chứng phù hợp. Trình bày riêng “ước lượng nhanh hiện tại” và “sửa lại quỹ đạo cũ”.
- **Bài tập trước khi học công thức:** vẽ robot quay trong lúc quét một bức tường, đánh dấu sai số của (a) không deskew, (b) chỉ IMU, (c) chỉ scan matching trên tường phẳng, (d) có loop sai. Với mỗi sai số, chỉ ra khối nào có thể sửa và khối nào không thể.
- **Đầu ra của chặng:** một trang giải thích giả thuyết thiết kế và bốn phản ví dụ trên; bảng “đầu vào → thông tin quan sát được → thông tin còn không quan sát được”. Đây là chuẩn để không học công thức rời rạc.

## 3. Chặng A — nền toán học và dữ liệu (học trước khi code pipeline)

### A0. Đọc kiến trúc và đóng băng mốc đối chiếu

- **Hiểu:** đường scan đi qua `imageProjection → featureExtraction → mapOptmization`; nhánh `imuPreintegration` chạy song song, nhận IMU và pose LiDAR hiệu chỉnh rồi trả dự đoán chuyển động. `cloud_info` nối các khối nào; những topic nào là cảm biến, trung gian, đầu ra. Phân biệt hình kiến trúc trong bài báo với hai đồ thị được README/source triển khai.
- **Tạo:** sơ đồ luồng dữ liệu một scan, bảng `topic → kiểu → frame → timestamp → bên tạo → bên dùng`, và danh sách điều kiện kích hoạt từng module.
- **Kiểm chứng:** truy được một trường trong `cloud_info` từ nơi sinh đến nơi dùng; ghi commit nguồn. Không suy từ tên topic rằng dữ liệu đã có đủ trường.
- **Nguồn:** [README, System architecture](https://github.com/TixiaoShan/LIO-SAM/blob/0be1fbe6275fb8366d5b800af4fc8c76a885c869/README.md), [cloud_info.msg](https://github.com/TixiaoShan/LIO-SAM/blob/0be1fbe6275fb8366d5b800af4fc8c76a885c869/msg/cloud_info.msg), [bài báo, III-A](https://arxiv.org/html/2007.00258).

### A1. Hình học 3D và quy ước ký hiệu

**Công thức phải tự suy ra [Paper (1) + Suy ra]:**

$$
T_{AB}=\begin{bmatrix}R_{AB}&t_{AB}\\0&1\end{bmatrix},\qquad
T_{AC}=T_{AB}T_{BC},\qquad
T_{BA}=T_{AB}^{-1},\qquad
p_W=T_{WB}T_{BL}p_L.
$$

Viết lại bốn phép trên cho một điểm tường và làm phép đổi ngược $p_L=T_{BL}^{-1}T_{WB}^{-1}p_W$. Ma trận $T_{BL}$ là ngoại chuẩn **LiDAR → body** theo quy ước của tài liệu này; phải đối chiếu chiều thực của tham số source trước khi gán.

- **Nền:** vector, ma trận quay `SO(3)`, quaternion, biến đổi cứng `SE(3)`, phép hợp thành/nghịch đảo, điểm trong frame LiDAR/IMU/body/world, roll–pitch–yaw chỉ là một cách biểu diễn. Phân biệt `T_WB` với `T_BW` và phép biến đổi điểm với pose robot.
- **Thuật toán cần nắm:** `p_W = T_WB · T_BL · p_L` (tọa độ thuần nhất); pose tương đối giữa hai thời điểm; cập nhật quay bằng `exp` và sai lệch quay bằng `log`. Trước khi viết Jacobian, làm rõ thứ tự biến và quy ước perturbation trái/phải.
- **Tạo:** bảng tên frame và chiều từng transform, thư viện phép biến đổi nhỏ hoặc giao diện dùng thư viện đã chọn; ví dụ tính bằng tay cho robot quay 90° với LiDAR lệch tâm.
- **Kiểm chứng:** biến đổi đi rồi về cho cùng điểm; chuỗi transform đúng dù robot quay và cảm biến lệch trục; quaternion chuẩn hóa. Sai chiều frame phải làm bài kiểm tra thất bại.
- **Nguồn:** [bài báo, III-A, (1)](https://arxiv.org/html/2007.00258), [README, Prepare IMU data](https://github.com/TixiaoShan/LIO-SAM/blob/0be1fbe6275fb8366d5b800af4fc8c76a885c869/README.md), [params.yaml](https://github.com/TixiaoShan/LIO-SAM/blob/0be1fbe6275fb8366d5b800af4fc8c76a885c869/config/params.yaml).

### A2. Tối ưu và xác suất đủ dùng cho SLAM

**Câu hỏi cần trả lời:** robot có nhiều thông tin chưa hoàn toàn khớp nhau. Ta tìm trạng thái nào giải thích các thông tin ấy tốt nhất, và xử lý thế nào khi một phép đo kém tin cậy? Đọc lần lượt A2.1 → A2.7; ở mỗi bước, hãy đọc **công thức → từng thành phần → tác dụng → ví dụ**.

Trong các ví dụ 1D dưới đây, $x$ chỉ là vị trí trên một trục để dễ tính tay. Trong LIO-SAM, trạng thái có thể gồm pose 3D, vận tốc và bias. Những con số minh họa không phải tham số của source gốc.

#### A2.1. Sai số đo và residual

**Công thức:**

$$
e=z-z_{\rm thật},
\qquad
r(x)=h(x)-z.
$$

- $z$: số cảm biến báo, ví dụ khoảng cách 2,1 m.
- $z_{\rm thật}$: giá trị đo đúng trong thực tế. Khi robot chạy, thường không biết nó nên không tính được $e$.
- $x$: trạng thái robot **đang thử**, ví dụ một pose giả định.
- $h(x)$: số cảm biến *đáng lẽ* phải báo nếu robot thật sự ở trạng thái $x$ và mô hình/map đúng.
- $r(x)$: **residual**, tức chênh lệch giữa dự đoán $h(x)$ và số đo $z$. Ta tính được nó cho từng $x$ đang thử. Quy ước ở đây là “dự đoán trừ đo”; đổi quy ước dấu phải đổi nhất quán ở Jacobian.

**Tác dụng:** residual biến câu “pose này có khớp dữ liệu không?” thành một con số để so sánh và tối ưu. Sai số đo $e$ nói về sự thật chưa biết; residual $r(x)$ nói về mức khớp của một giả thuyết có thể kiểm tra.

**Ví dụ:** tường thật cách 2,0 m, LiDAR báo $z=2{,}1$ m, vậy sai số đo là $e=0{,}1$ m nếu ta có ground truth. Với một pose thử, mô hình dự đoán $h(x)=2{,}3$ m thì $r(x)=0{,}2$ m. Thử pose khác cho $h(x)=2{,}05$ m thì $r(x)=-0{,}05$ m; độ lệch nhỏ hơn. Residual bằng 0 vẫn chưa chứng minh pose đúng nếu map hoặc tương ứng hình học sai.

**Trong LIO-SAM:** scan matching có residual hình học giữa điểm LiDAR và đường/mặt của local map (mục C2). Khoảng cách điểm–mặt không âm là **độ lớn** của residual; khi cần đạo hàm, chiều pháp tuyến giúp định nghĩa residual có dấu.

#### A2.2. Phương sai, hiệp phương sai và thông tin

**Công thức:**

$$
\sigma^2=\mathbb E[(e-\mathbb E[e])^2],
\qquad
\Sigma=\mathbb E[(e-\mathbb E[e])(e-\mathbb E[e])^\top],
\qquad
w=\frac{1}{\sigma^2},\quad \Omega=\Sigma^{-1}.
$$

- $\mathbb E[\cdot]$: giá trị trung bình kỳ vọng qua nhiều lần đo trong cùng điều kiện.
- $e$: sai số đo ngẫu nhiên. Công thức $\sigma^2$ nói sai số **phân tán quanh trung bình** nhiều đến đâu; nó không cho biết một lần đo cụ thể sai bao nhiêu.
- $\sigma$: độ lệch chuẩn, cùng đơn vị với số đo; $\sigma^2$ là **phương sai**, có đơn vị bình phương. Với khoảng cách, lần lượt là m và m².
- $\Sigma$: **ma trận hiệp phương sai** khi sai số có nhiều thành phần, ví dụ $(x,y,z)$. Đường chéo là phương sai từng thành phần; phần tử ngoài đường chéo diễn tả hai thành phần sai lệch có liên hệ.
- $w$ hoặc $\Omega$: **trọng số/thông tin** dùng trong tối ưu. Chúng là nghịch đảo của phương sai/hiệp phương sai trong mô hình Gaussian; phương sai nhỏ cho trọng số lớn. Phép đảo $\Sigma^{-1}$ đòi hỏi một ma trận hiệp phương sai hợp lệ.

**Tác dụng:** cho thuật toán biết residual nào nên có ảnh hưởng mạnh hơn. Hai residual cùng lớn 0,2 m nhưng xuất phát từ hai nguồn có độ nhiễu khác nhau không nên bị xử lý như nhau. Đặt covariance sai có thể khiến thuật toán quá tin dữ liệu xấu.

**Ví dụ:** phép đo A có $\sigma_A=0{,}1$ m nên $\sigma_A^2=0{,}01$ m² và $w_A=100$ m⁻². Phép đo B có $\sigma_B=1$ m nên $w_B=1$ m⁻². Theo mô hình này, residual của A được phạt mạnh gấp 100 lần residual cùng độ lớn của B. Đây là **độ tin cậy được gán từ mô hình nhiễu**, không phải bằng chứng A luôn đúng; bias có thể làm A sai có hệ thống.

#### A2.3. Bình phương tối thiểu có trọng số

**Công thức 1D [Suy ra]:**

$$
F(x)=\sum_{i=1}^{N}w_i\,r_i(x)^2
    =\sum_{i=1}^{N}\frac{(x-z_i)^2}{\sigma_i^2},
\qquad
x^*=\arg\min_x F(x).
$$

- $N$: số phép đo đang kết hợp.
- $r_i(x)=x-z_i$: residual của phép đo thứ $i$ trong ví dụ các cảm biến cùng đo vị trí $x$.
- $w_i=1/\sigma_i^2$: trọng số của phép đo $i$.
- $F(x)$: tổng “mức không khớp” có xét độ tin cậy; nhỏ hơn nghĩa là các phép đo được giải thích tốt hơn **theo mô hình đã chọn**.
- $\arg\min_x$: tìm **giá trị $x$** khiến $F(x)$ nhỏ nhất, không phải lấy giá trị nhỏ nhất của $F$ làm pose.

**Tác dụng:** biến nhiều phép đo mâu thuẫn thành một câu hỏi có thể giải: chọn $x$ sao cho tổng sai lệch có trọng số nhỏ nhất. Bình phương làm residual dương/âm không triệt tiêu nhau và phạt mạnh sai lệch lớn.

**Ví dụ tự tính:** $z_1=2$ m, $\sigma_1=0{,}1$ m; $z_2=3$ m, $\sigma_2=1$ m. Khi đó $w_1=100$, $w_2=1$ và

$$
F(x)=100(x-2)^2+(x-3)^2,\qquad
\frac{dF}{dx}=200(x-2)+2(x-3)=0,
$$

$$
x^*=\frac{100\cdot2+1\cdot3}{100+1}
    =\frac{203}{101}\approx2{,}0099\ {\rm m}.
$$

Nghiệm gần 2 m vì phép đo đầu được gán độ bất định nhỏ hơn. Nếu đổi hai giá trị $\sigma$, nghiệm phải nghiêng về 3 m. Ví dụ này giả định hai phép đo độc lập, không có outlier và cùng đo đúng đại lượng $x$.

#### A2.4. Prior và MAP: đưa thông tin có trước vào bài toán

**Công thức Bayes và MAP [Paper III-A ở mức ý tưởng]:**

$$
p(x\mid z)\propto p(z\mid x)\,p(x),
\qquad
x_{\rm MAP}=\arg\max_x p(x\mid z).
$$

- $p(x)$: **prior**, tức mức tin vào trạng thái $x$ *trước khi* nhận phép đo mới. Prior có thể đến từ trạng thái/dự đoán trước; nó cũng có sai số.
- $p(z\mid x)$: **likelihood**, tức phép đo $z$ có hợp lý đến đâu nếu trạng thái đang thử là $x$.
- $p(x\mid z)$: **posterior**, mức tin vào $x$ *sau khi* biết $z$.
- $\propto$: “tỷ lệ với”; bỏ hệ số chuẩn hóa không phụ thuộc $x$ vì nó không làm đổi vị trí cực đại.
- $x_{\rm MAP}$: trạng thái làm posterior lớn nhất.

**Tác dụng:** cho phép kết hợp thông tin cũ với phép đo mới trong một mô hình xác suất. Khi residual được mô hình bằng Gaussian, lấy âm log biến phép nhân các xác suất thành tổng bình phương có trọng số:

$$
x^*=\arg\min_x\frac12\sum_i
r_i(x)^\top\Sigma_i^{-1}r_i(x).
$$

Ở công thức tổng quát, $i$ chạy qua các factor; $r_i(x)$ có thể là vector; $\Sigma_i$ là covariance của **chính residual đó**; $\tfrac12$ tiện cho đạo hàm và không đổi vị trí nghiệm. Các đơn vị khác nhau, như m và rad, được đưa về mức ảnh hưởng tương ứng thông qua covariance phù hợp.

**Ví dụ 1D [Suy ra]:** giả sử đã quy hai nguồn về cùng trục vị trí. Dự đoán ban đầu $x_0=2{,}5$ m với $\sigma_0=0{,}5$ m; phép đo mới $z=2{,}0$ m với $\sigma_z=0{,}1$ m:

$$
x_{\rm MAP}=\arg\min_x
\left[\frac{(x-x_0)^2}{\sigma_0^2}
     +\frac{(x-z)^2}{\sigma_z^2}\right]
=\frac{4\cdot2{,}5+100\cdot2{,}0}{4+100}
\approx2{,}019\ {\rm m}.
$$

Ở đây prior có trọng số $1/0{,}5^2=4$, phép đo mới có trọng số $1/0{,}1^2=100$. Nghiệm gần phép đo mới hơn. **Đây là ví dụ để học MAP, không phải công thức scan-to-map của LIO-SAM gốc**: source dùng dự đoán IMU làm khởi tạo scan matching và một graph IMU riêng nhận pose LiDAR correction (xem D2–D3).

#### A2.5. Jacobian và Gauss–Newton: biết phải sửa biến theo hướng nào

**Công thức đạo hàm và xấp xỉ [Suy ra]:**

$$
J_{ij}=\frac{\partial r_i}{\partial x_j},
\qquad
r(x_k+\delta)\approx r(x_k)+J\delta.
$$

- $r_i$: residual thứ $i$; $x_j$: biến thứ $j$. Nếu có $m$ residual và $n$ biến, Jacobian $J$ có $m$ hàng và $n$ cột.
- $x_k$: nghiệm đang thử ở vòng lặp thứ $k$; $\delta$: bước sửa nhỏ cho nghiệm đó.
- $J\delta$: dự đoán residual sẽ thay đổi thế nào nếu sửa biến bằng $\delta$. Phép xấp xỉ chỉ đáng tin **gần** $x_k$.

**Tác dụng:** thay bài toán phi tuyến khó giải bằng một bài toán gần tuyến tính tại pose hiện tại. Với covariance gom thành ma trận trọng số $W$, bước Gauss–Newton được tìm qua

$$
(J^\top WJ)\delta=-J^\top Wr,\qquad
x_{k+1}=x_k\boxplus\delta.
$$

- $r$ và $J$ được tính tại $x_k$; $W$ chứa trọng số của các residual.
- $J^\top WJ$ tóm tắt các hướng mà phép đo cung cấp thông tin. Nếu nó thiếu hạng, một số hướng không được quan sát đủ.
- $\delta$ là bước sửa tìm được; $\boxplus$ nghĩa là **cách cập nhật trạng thái**. Trong ví dụ một số, đó là cộng bình thường; với pose 3D phải định nghĩa phép cập nhật quay, nên không tự thay $\boxplus$ bằng cộng từng góc.

**Ví dụ một vòng lặp:** muốn $x^2=4$, ta đặt $r(x)=x^2-4$. Bắt đầu $x_0=3$ thì $r=5$ và $J=dr/dx=2x_0=6$. Với $W=1$, công thức cho $36\delta=-30$, vậy $\delta=-5/6$ và $x_1\approx2{,}167$. Tính lại $r,J$ tại $x_1$ rồi lặp sẽ tiến gần nghiệm 2. Jacobian cho biết bước sửa, không phải một phép đo mới.

#### A2.6. LM, suy biến và outlier: khi giải phương trình chưa đủ

**Levenberg–Marquardt (LM), dạng minh họa:**

$$
(J^\top WJ+\lambda I)\delta=-J^\top Wr,\qquad\lambda>0.
$$

- $\lambda I$ là hạng **damping**; $\lambda$ quyết định mức hạn chế bước sửa. Dạng solver thực tế có thể dùng cách scale khác với $I$.
- **Tác dụng:** nếu xấp xỉ tuyến tính tại $x_k$ chưa tốt, thử bước thận trọng hơn; solver điều chỉnh damping theo việc bước thử có giảm chi phí thật hay không.
- **Ví dụ:** ở bài $x^2=4$ với $x_0=3$, Gauss–Newton cho $\delta=-30/36\approx-0{,}833$. Nếu minh họa LM bằng $\lambda=36$, bước là $\delta=-30/(36+36)\approx-0{,}417$: nhỏ hơn. Không được kết luận tên hàm LMOptimization() của LIO-SAM chứng minh source dùng damping này; phải đọc phép tính trong hàm.

**Suy biến, ví dụ hai biến [Suy ra]:** nếu chỉ có một bức tường $x=2$ và residual điểm–mặt là $r(x,y)=x-2$, thì

$$
J=\begin{bmatrix}1&0\end{bmatrix},
\qquad
J^\top J=\begin{bmatrix}1&0\\0&0\end{bmatrix}.
$$

Cột theo $y$ bằng 0: dịch dọc theo tường không đổi residual, nên dữ liệu này không cho biết $y$ là bao nhiêu. Ma trận $J^\top J$ thiếu hạng; damping có thể giúp bước số ổn định nhưng **không tạo thông tin hình học về $y$**. Trong LIO-SAM, cần xem code nhận biết và xử lý các hướng suy biến trong scan matching.

**Outlier:** một điểm trên người đi ngang có thể bị ghép nhầm với tường tĩnh. Nó tạo residual không đại diện cho pose robot; bình phương residual lớn có thể kéo nghiệm sai. Tùy bước thuật toán, ta kiểm tra hình học trước khi nhận tương ứng, đặt ngưỡng hoặc dùng hàm mất mát bền vững. Residual lớn cũng có thể do pose khởi tạo, thời gian hoặc ngoại chuẩn sai; phải tìm nguyên nhân trước khi loại dữ liệu.

#### A2.7. Tự kiểm tra trước khi học C2, C3 và D2

1. Giải thích khác nhau giữa **sai số đo** (thường không biết khi chạy) và **residual** (tính được từ pose đang thử).
2. Tính lại ví dụ $z_1=2$ m, $z_2=3$ m; đổi hai $\sigma$ và dự đoán nghiệm sẽ nghiêng về đâu trước khi bấm máy.
3. Với $r(x)=x^2-4$, tự tính $J$ và bước Gauss–Newton từ $x_0=3$; thử thêm một $\lambda$ để thấy bước LM thay đổi ra sao.
4. Từ $J=[1\ 0]$, giải thích vì sao một bức tường phẳng không xác định được chuyển động dọc tường qua residual này.
5. Chỉ ra trong luồng LIO-SAM: residual điểm–đường/mặt ở C2, tối ưu pose ở C3, prior/correction và các graph ở D2–D3.

**Đầu ra để code sau A2:** một bài toán 1D/2D nhỏ có weighted least squares, một vòng Gauss–Newton và trường hợp suy biến; kiểm chứng nghiệm bằng tính tay và Jacobian bằng sai phân hữu hạn. Chưa cần viết iSAM2 ở bài này.

**Đọc gốc:** [LIO-SAM, III-A và III-C3](https://arxiv.org/html/2007.00258) nêu MAP và tối ưu LiDAR; [GTSAM, Factor Graphs and GTSAM](https://gtsam.org/tutorials/intro.html) giải thích biến/factor/prior; [Ceres, Solving Non-linear Least Squares](https://ceres-solver.readthedocs.io/latest/nnls_solving.html) giải thích Jacobian, tuyến tính hóa và bước tối ưu. Đối chiếu [LMOptimization() trong source LIO-SAM](https://github.com/TixiaoShan/LIO-SAM/blob/0be1fbe6275fb8366d5b800af4fc8c76a885c869/src/mapOptmization.cpp#L1158-L1279) khi học C3.

### A3. Hợp đồng dữ liệu, thời gian và ngoại chuẩn

**Công thức cần kiểm tra [Suy ra]:** $t_k=t_{\text{scan}}+\tau_k$; nếu hai đồng hồ có lệch cố định theo quy ước đã chọn, $t^{I}=t^{L}+\Delta t_{LI}$. Với mọi điểm $k$, khoảng IMU phải bao phủ $t_k$; một $t_{\text{scan}}$ không thể thay thế mọi $\tau_k$. Ghi rõ đơn vị của $\tau_k$, độ phân giải timestamp và ý nghĩa $\Delta t_{LI}$ trước khi nội suy.

- **Nền:** `header.stamp` của scan, thời gian tương đối từng điểm, `ring`, thứ tự điểm, hàng đợi IMU, thời gian bắt đầu/kết thúc scan, lệch đồng hồ và độ trễ truyền. `ring` là kênh tia, không phải chỉ số thời gian. Kiểm tra `NaN`, phạm vi đo, mật độ điểm, thứ tự timestamp và frame trước thuật toán.
- **Thuật toán cần nắm:** nội suy chỉ hợp lệ khi có mẫu bao quanh thời điểm cần tính; ngoại chuẩn tịnh tiến/quay và biến đổi acceleration, gyro, orientation; khác nhau giữa hiệu chuẩn thời gian, đồng bộ thông điệp và deskew.
- **Tạo:** đặc tả đầu vào `TimedPoint/Scan/ImuSample` với đơn vị SI, frame, mốc thời gian; bộ kiểm tra cho trường thiếu/không hợp lệ; bảng phản ứng `xử lý / bỏ scan / báo lỗi`. Quyết định cách đồng bộ và nguồn timestamp trước khi tích hợp.
- **Kiểm chứng:** ca dữ liệu IMU thiếu đầu/cuối scan, timestamp lùi, `ring` ngoài phạm vi, điểm không hữu hạn và ngoại chuẩn đảo chiều; mọi ca phải có kết quả rõ ràng.
- **Giới hạn dự án:** cloud `/points` của mô phỏng hiện mới có `x,y,z,intensity`; nó không đủ để kiểm chứng nguyên bản deskew theo từng điểm và phép xếp `ring` của LiDAR quay. Có thể dùng dữ liệu tổng hợp hoặc bag hợp lệ cho các bài này; không gán `time` giả rồi coi là đo thật. Chi tiết cảm biến mô phỏng cần được kiểm tra lại khi chuyển sang thực nghiệm.
- **Nguồn:** [README, Prepare lidar/IMU data](https://github.com/TixiaoShan/LIO-SAM/blob/0be1fbe6275fb8366d5b800af4fc8c76a885c869/README.md), [imageProjection.cpp: cachePointCloud, deskewInfo](https://github.com/TixiaoShan/LIO-SAM/blob/0be1fbe6275fb8366d5b800af4fc8c76a885c869/src/imageProjection.cpp), [params.yaml](https://github.com/TixiaoShan/LIO-SAM/blob/0be1fbe6275fb8366d5b800af4fc8c76a885c869/config/params.yaml).

## 4. Chặng B — từ phép đo đến đặc trưng hình học

### B1. Tích lũy chuyển động IMU trong một scan

**Công thức học [Paper (2), (6), rời rạc hóa]:**

$$
\hat\omega_k=\omega_k+b_g+n_g,\qquad
R_{k+1}\approx R_k\operatorname{Exp}\!\big((\hat\omega_k-b_g)\Delta t_k\big).
$$

Tự tính scan dài $0{,}1\,s$ khi robot quay đều $90^\circ/s$: điểm ở giữa scan lệch khoảng $4{,}5^\circ$ so với đầu scan. Source deskew tích lũy các thành phần góc từ số đo gyro đã đổi frame; nó không gọi GTSAM preintegration hay áp trực tiếp bias ước lượng của graph IMU trong hàm này. Công thức $\operatorname{Exp}$ ở đây là mô hình hình học để học, **không phải chép nguyên phép tính source**.

- **Nền:** gyro đo tốc độ góc, accelerometer đo gia tốc riêng, trọng lực, bias/nhiễu, tích phân rời rạc và nội suy theo timestamp. Tách luồng này khỏi **IMU preintegration factor** ở chặng D.
- **Thuật toán cần nắm:** tính hướng tương đối từ mẫu IMU phủ toàn scan; chọn mốc đầu scan và tra hướng ở thời gian từng điểm; hiểu giới hạn khi chỉ sửa méo quay.
- **Tạo:** hàm trả chuyển động tương đối theo thời gian từng điểm; dữ liệu tổng hợp robot đứng yên, quay đều, quay tăng tốc.
- **Kiểm chứng:** khi đứng yên transform gần đồng nhất; quay đều cho đúng góc tại đầu/giữa/cuối; thiếu đoạn IMU phải bị phát hiện.
- **Nguồn:** [bài báo, III-A/III-B](https://arxiv.org/html/2007.00258); [imageProjection.cpp: imuDeskewInfo, deskewPoint](https://github.com/TixiaoShan/LIO-SAM/blob/0be1fbe6275fb8366d5b800af4fc8c76a885c869/src/imageProjection.cpp).

### B2. Deskew từng điểm

**Công thức hình học [Suy ra; đối chiếu với deskewPoint()]:**

$$
p_L^{\,t_{\rm ref}}=
T_{WL}(t_{\rm ref})^{-1}T_{WL}(t_k)\,p_L^{\,t_k},
\qquad t_k=t_0+\tau_k.
$$

Điểm tại thời điểm $t_k$ được biểu diễn lại như thể đo ở frame LiDAR tại $t_{\rm ref}$. Nếu robot không chuyển động, tích hai transform là đơn vị. Source gốc dùng transform của **điểm hợp lệ đầu tiên** làm mốc trong hàm deskew, và thành phần tịnh tiến mặc định bằng 0; không được âm thầm coi mốc đó luôn đúng bằng timestamp đầu thông điệp.

- **Nền:** một scan LiDAR quay được đo tuần tự; pose thay đổi trong thời gian đó. Chọn rõ frame và thời điểm tham chiếu của scan.
- **Thuật toán cần nắm:** với điểm đo tại `t_i`, đưa nó về frame ở `t_ref` bằng chuyển động tương đối; phân biệt khử méo quay và tịnh tiến. Trong source gốc tại commit đối chiếu, đường `findPosition()` cho tịnh tiến đang bị chú thích, nên cần học hành vi code thực tế bên cạnh mô tả ý tưởng trong bài báo.
- **Tạo:** scan đã deskew và thống kê tỷ lệ điểm không deskew được; giao diện có chỗ biểu diễn mức hỗ trợ `rotation-only`/đầy đủ, nhưng chưa chốt chính sách cho dự án.
- **Kiểm chứng:** quét một tường phẳng khi robot quay: điểm sau deskew gần cùng mặt phẳng hơn dữ liệu thô; khi chuyển động bằng 0 đầu ra gần như không đổi; không đánh giá hiệu quả deskew bằng scan mô phỏng chụp đồng thời.
- **Nguồn:** [README, point time](https://github.com/TixiaoShan/LIO-SAM/blob/0be1fbe6275fb8366d5b800af4fc8c76a885c869/README.md), [imageProjection.cpp: findPosition, deskewPoint](https://github.com/TixiaoShan/LIO-SAM/blob/0be1fbe6275fb8366d5b800af4fc8c76a885c869/src/imageProjection.cpp).

### B3. Chiếu scan thành range image và lấy lại cloud có cấu trúc

**Công thức [Source, nhánh LiDAR quay Velodyne/Ouster]:**

$$
r=\sqrt{x^2+y^2+z^2},\quad
\text{row}=\text{ring},\quad
\theta=\operatorname{atan2}(x,y)\frac{180}{\pi},\quad
\text{col}=-\operatorname{round}\!\left(\frac{\theta-90}{360/H}\right)+\frac H2.
$$

$H$ là số cột ngang; source xử lý cột vượt biên sau phép tính và bỏ điểm nếu ô đã có giá trị. Công thức cột này phụ thuộc quy ước trục và kiểu cảm biến trong code; không suy ra nó đúng cho mọi LiDAR.

- **Nền:** hàng `ring`, cột theo góc quét, khoảng cách của điểm, ô trống/nhiều điểm một ô, dải quét ngang và dọc. Tính riêng đầu ra “ảnh khoảng cách” với ảnh RGB.
- **Thuật toán cần nắm:** lọc khoảng cách; chiếu điểm vào ô; chọn điểm khi trùng ô; ghi chỉ số điểm hợp lệ để tìm hàng xóm; lấy lại cloud có thứ tự cho bước tính độ gồ ghề.
- **Tạo:** ma trận range và metadata mỗi ring/điểm, cùng cloud đã deskew và sắp đúng thứ tự. Định nghĩa rõ đầu vào của module B4.
- **Kiểm chứng:** dữ liệu tổng hợp vài ring cho đúng hàng/cột; điểm ngoài tầm, `NaN`, ô trống và cạnh biên không làm lệch chỉ số; xem lại phân bố điểm trên từng ring.
- **Nguồn:** [imageProjection.cpp: projectPointCloud, cloudExtraction](https://github.com/TixiaoShan/LIO-SAM/blob/0be1fbe6275fb8366d5b800af4fc8c76a885c869/src/imageProjection.cpp), [cloud_info.msg](https://github.com/TixiaoShan/LIO-SAM/blob/0be1fbe6275fb8366d5b800af4fc8c76a885c869/msg/cloud_info.msg).

### B4. Chọn đặc trưng cạnh và mặt

**Công thức [Source, calculateSmoothness()]:**

$$
d_i=\sum_{\substack{k=-5\\k\ne0}}^{5}r_{i+k}-10r_i,
\qquad c_i=d_i^2.
$$

$r_i$ ở đây là **range của điểm đã xếp thứ tự**, không phải residual tối ưu. $c_i$ lớn là ứng viên cạnh, nhỏ là ứng viên mặt; còn phải kiểm tra che khuất, biên ring và chọn theo vùng trước khi dùng làm feature.

- **Nền:** biến thiên khoảng cách theo lân cận trong cùng ring, độ gồ ghề, điểm bị che khuất, mặt nhẵn, cạnh; voxel để giảm mật độ mặt.
- **Thuật toán cần nắm:** tính smoothness; đánh dấu điểm bị che khuất/song song tia; chia vùng trước khi xếp hạng; chọn cạnh với roughness lớn và mặt với roughness nhỏ; tránh chọn dồn một cụm điểm.
- **Tạo:** `FeatureSet` gồm cloud cạnh, cloud mặt và thống kê số điểm bị loại theo lý do.
- **Kiểm chứng:** mô hình tường phẳng + mép cửa cho phân bố đặc trưng dễ giải thích; kiểm tra trường hợp tường rất xa, cảnh quá ít điểm và nhiễu; so số điểm/thời gian với code gốc trên cùng scan.
- **Nguồn:** [bài báo, III-C](https://arxiv.org/html/2007.00258), [featureExtraction.cpp: calculateSmoothness, markOccludedPoints, extractFeatures](https://github.com/TixiaoShan/LIO-SAM/blob/0be1fbe6275fb8366d5b800af4fc8c76a885c869/src/featureExtraction.cpp).

## 5. Chặng C — LiDAR odometry và bản đồ cục bộ

### C1. Lưu keyframe và tạo local map

**Công thức [Paper III-C1 + Suy ra]:** với tập chỉ số keyframe được chọn $S_i$,

$$
\mathcal M_i^e=\operatorname{Voxel}_e\!\left(\bigcup_{j\in S_i}T_{W B_j}\mathcal F_j^e\right),
\qquad
\mathcal M_i^p=\operatorname{Voxel}_p\!\left(\bigcup_{j\in S_i}T_{W B_j}\mathcal F_j^p\right).
$$

$\mathcal F_j^e,\mathcal F_j^p$ là feature trong frame body/keyframe; nếu chúng thực tế nằm trong frame LiDAR thì phải chèn $T_{BL}$ theo hợp đồng dữ liệu. Bài báo chọn $S_i$ bằng số keyframe mới nhất; source chọn theo bán kính và thời gian. Điều kiện keyframe có thể viết dạng kiểm tra $\|\Delta p\|>d_{\rm th}$ **hoặc** thay đổi góc vượt ngưỡng; phải xem source kiểm tra từng thành phần roll/pitch/yaw ra sao.

- **Nền:** scan hiện tại khác keyframe; feature cloud gắn với pose keyframe; world-frame cloud có thể phải dựng lại khi pose keyframe đổi. Voxel downsampling khác bản đồ occupancy xác suất.
- **Thuật toán cần nắm:** quyết định giữ keyframe theo dịch chuyển/góc; lấy keyframe lân cận, biến đổi cạnh/mặt vào cùng frame, voxel hóa, xây chỉ mục tìm hàng xóm; xử lý bootstrap khi chưa có map. Bài báo mô tả cửa sổ số keyframe cố định, còn source tại commit đối chiếu chọn theo bán kính, lấy mẫu pose và bổ sung keyframe mới theo thời gian: phải ghi rõ hành vi nào đang tái hiện.
- **Tạo:** kho keyframe và hai local feature maps (cạnh, mặt), với quy tắc rõ cho cache và invalidation sau đóng vòng.
- **Kiểm chứng:** đưa một keyframe qua pose đã biết cho điểm đúng vị trí; số keyframe và kích thước local map có giới hạn; sau thay pose cũ, map dựng lại đúng.
- **Nguồn:** [bài báo, III-C1](https://arxiv.org/html/2007.00258), [mapOptmization.cpp: extractSurroundingKeyFrames, saveFrame](https://github.com/TixiaoShan/LIO-SAM/blob/0be1fbe6275fb8366d5b800af4fc8c76a885c869/src/mapOptmization.cpp).

### C2. Tương ứng hình học điểm–đường, điểm–mặt

**Công thức khoảng cách [Paper (10), (11), viết theo ký hiệu thống nhất]:** gọi $q=T_{WB}p$ là điểm scan đã đưa vào W; $a,b$ xác định đường map; $a,b,c$ xác định một mảng mặt phẳng map.

$$
d_{\rm edge}(q)=\frac{\|(q-a)\times(q-b)\|}{\|a-b\|},\qquad
d_{\rm plane}(q)=\frac{|(q-a)^\top[(b-a)\times(c-a)]|}
{\|(b-a)\times(c-a)\|}.
$$

Mẫu số gần 0 nghĩa là tương ứng hình học không hợp lệ. Khi tối ưu, cần chọn residual có dấu/đạo hàm phù hợp và kiểm tra cách source **fit** đường, mặt từ hàng xóm; không lấy ba điểm tùy ý rồi coi đã tái hiện source.

- **Nền:** k-nearest-neighbors, PCA/eigenvalue để xác nhận cấu trúc đường hoặc mặt, khoảng cách điểm–đường/mặt, điều kiện suy biến, outlier.
- **Thuật toán cần nắm:** từ pose khởi tạo, đưa điểm scan sang map; tìm hàng xóm map; chỉ tạo residual nếu cấu trúc hình học đáng tin; phân biệt tương ứng cạnh và mặt.
- **Tạo:** tập residual kèm số tương ứng được chấp nhận/từ chối và lý do; interface riêng khỏi optimizer.
- **Kiểm chứng:** trên đường/mặt tổng hợp, residual gần 0 khi pose đúng và tăng theo perturbation; trường hợp hàng xóm không tạo được đường/mặt phải bị loại.
- **Nguồn:** [bài báo, III-C2, (10)–(11)](https://arxiv.org/html/2007.00258), [mapOptmization.cpp: cornerOptimization, surfOptimization](https://github.com/TixiaoShan/LIO-SAM/blob/0be1fbe6275fb8366d5b800af4fc8c76a885c869/src/mapOptmization.cpp).

### C3. Tối ưu pose scan-to-map

**Công thức học cho optimizer [Suy ra từ residual Paper III-C3]:**

$$
T^*=\arg\min_{T\in SE(3)}
\left[\sum_{k\in E}w_k^e\,r_{e,k}(T)^2+
      \sum_{k\in P}w_k^p\,r_{p,k}(T)^2\right],
\qquad
(J^\top WJ)\delta=-J^\top Wr.
$$

$E,P$ là tập tương ứng cạnh/mặt đã qua kiểm tra; $r$ là residual có dấu, còn $d_{\rm edge},d_{\rm plane}$ ở C2 là **khoảng cách không âm**. Bài báo (11) viết tổng khoảng cách; biểu thức bình phương ở đây là bài toán chuẩn để học linearization, **không được tuyên bố là chép nguyên (11) hoặc mã C++**. Source có bước chiếu bỏ hướng suy biến trong hàm tên LMOptimization; phải đọc đúng phép tính trước khi chọn solver.

- **Nền:** 6 bậc tự do, Jacobian residual theo pose, linearization/lặp, stopping criteria, nghiệm suy biến; pose khởi tạo từ IMU là điểm bắt đầu chứ không phải phép đo hình học cuối cùng.
- **Thuật toán cần nắm:** lặp tìm tương ứng → tối ưu pose → cập nhật tương ứng; cách source giới hạn pose theo IMU/độ cao hoặc thông số cấu hình; khi nào bỏ tối ưu vì thiếu đặc trưng. Hàm `LMOptimization()` trong source giải hệ tuyến tính và xử lý suy biến; tên hàm không đủ để kết luận có bước damping của Levenberg–Marquardt.
- **Tạo:** `RegistrationResult` gồm pose, trạng thái hội tụ/suy biến, residual, số vòng lặp, thời gian; không trả “pose tốt” khi thực tế thiếu ràng buộc.
- **Kiểm chứng:** scan tổng hợp bị dịch/quay một lượng biết trước hội tụ khi khởi tạo đủ gần; ca hành lang/một mặt phẳng báo suy biến; kiểm tra Jacobian bằng sai phân hữu hạn.
- **Nguồn:** [bài báo, III-C2/III-C3](https://arxiv.org/html/2007.00258), [mapOptmization.cpp: updateInitialGuess, scan2MapOptimization, LMOptimization](https://github.com/TixiaoShan/LIO-SAM/blob/0be1fbe6275fb8366d5b800af4fc8c76a885c869/src/mapOptmization.cpp).

### C4. Từ pose LiDAR đến ràng buộc keyframe

**Công thức [Paper (12), diễn giải bằng phép nghịch đảo SE(3)]:**

$$
Z^{L}_{ij}=T_{W B_i}^{-1}T_{W B_j},\qquad
r^{L}_{ij}=\operatorname{Log}\!\left((Z^{L}_{ij})^{-1}T_{W B_i}^{-1}T_{W B_j}\right).
$$

$Z^L_{ij}$ được tính một lần từ hai pose LiDAR đã khớp và **giữ cố định khi tạo factor**; các $T_{W B_i},T_{W B_j}$ trong residual là biến của đồ thị, có thể đổi sau tối ưu. Residual bằng 0 nếu hai biến pose khớp phép đo. Ký hiệu $T_i^\top T_j$ trong bài báo (12) cần được đọc theo quy ước của tác giả; với **ma trận thuần nhất SE(3)**, triển khai phải dùng nghịch đảo $T_i^{-1}$, không dùng phép chuyển vị toàn ma trận.

- **Nền:** pose toàn cục và chuyển động tương đối; prior của keyframe đầu; độ tin cậy của odometry; chọn keyframe sau khi đã có pose.
- **Thuật toán cần nắm:** tính `T_i^{-1} T_j` đúng chiều và chỉ thêm node/factor khi keyframe được chấp nhận. Phân biệt pose scan đang xử lý, pose keyframe lưu trong đồ thị và pose đã sửa sau loop. Mapping có thể xử lý một scan và công bố pose hiệu chỉnh dù scan đó không trở thành keyframe.
- **Tạo:** chuỗi keyframe cùng LiDAR odometry factors; đầu ra LiDAR pose đã hiệu chỉnh để gửi ngược cho nhánh IMU.
- **Kiểm chứng:** quỹ đạo tổng hợp đi một vòng cho pose tương đối và phép hợp thành đúng; không nhân đôi keyframe khi scan đứng yên; đầu ra liên tục qua thời điểm bỏ keyframe.
- **Nguồn:** [bài báo, III-C3](https://arxiv.org/html/2007.00258), [mapOptmization.cpp: saveKeyFramesAndFactor, addOdomFactor](https://github.com/TixiaoShan/LIO-SAM/blob/0be1fbe6275fb8366d5b800af4fc8c76a885c869/src/mapOptmization.cpp).

## 6. Chặng D — kết hợp chặt IMU và đồ thị

### D1. Mô hình IMU và tích phân trước

**Công thức [Paper (2)–(9)]:**

$$
\hat\omega=\omega+b_g+n_g,\qquad
\hat a=R_{BW}(a_W-g_W)+b_a+n_a,
$$

$$
R_{k+1}=R_k\operatorname{Exp}((\hat\omega_k-b_g)\Delta t),\quad
v_{k+1}=v_k+g_W\Delta t+R_k(\hat a_k-b_a)\Delta t,
$$

$$
p_{k+1}=p_k+v_k\Delta t+\tfrac12g_W\Delta t^2+
\tfrac12R_k(\hat a_k-b_a)\Delta t^2.
$$

Sau khi hiểu tích phân từng mẫu, tự suy ra đại lượng tương đối giữa $i,j$:

$$
\Delta R_{ij}=R_i^\top R_j,\quad
\Delta v_{ij}=R_i^\top(v_j-v_i-g_W\Delta t_{ij}),\quad
\Delta p_{ij}=R_i^\top(p_j-p_i-v_i\Delta t_{ij}-\tfrac12g_W\Delta t_{ij}^2).
$$

Để **tính** các delta đó từ mẫu IMU, bắt đầu $\Delta R=I,\Delta v=0,\Delta p=0$ tại $i$, rồi lặp với mỗi mẫu $k$ giữa $i$ và $j$:

$$
\Delta R_{k+1}=\Delta R_k\operatorname{Exp}((\hat\omega_k-b_g)\Delta t_k),\quad
\Delta v_{k+1}=\Delta v_k+\Delta R_k(\hat a_k-b_a)\Delta t_k,
$$

$$
\Delta p_{k+1}=\Delta p_k+\Delta v_k\Delta t_k+
\tfrac12\Delta R_k(\hat a_k-b_a)\Delta t_k^2.
$$

Đây là **[Suy ra]** ở dạng Euler danh định để người học tự code thử, chưa gồm truyền covariance/Jacobian bias. Một residual IMU ở mức khái niệm là

$$
r_R=\operatorname{Log}(\Delta R_{ij}^{-1}R_i^\top R_j),\quad
r_v=R_i^\top(v_j-v_i-g_W\Delta t_{ij})-\Delta v_{ij},\quad
r_p=R_i^\top(p_j-p_i-v_i\Delta t_{ij}-\tfrac12g_W\Delta t_{ij}^2)-\Delta p_{ij}.
$$

Các phương trình động học trên lược bỏ hạng nhiễu khi tính **dự đoán danh định**; lúc làm factor phải đưa noise, covariance và bias correction trở lại. Paper dẫn [công trình preintegration của Forster và cộng sự](https://arxiv.org/abs/1512.02363) cho phần suy dẫn đầy đủ; khi dùng GTSAM phải đối chiếu cả [API preintegration](https://gtsam.org/doxygen/a04695.html) và phiên bản thư viện thực tế.

- **Nền:** trạng thái `R,p,v,b_g,b_a`, trọng lực, gyro/acceleration bias, white noise, random walk; phân biệt “tích lũy hướng cho deskew” với preintegration tạo factor giữa hai trạng thái.
- **Thuật toán cần nắm:** tích lũy `ΔR, Δv, Δp` giữa hai lần tạo trạng thái trong graph IMU (theo source là hai lần LiDAR correction, không nhất thiết là hai keyframe); ảnh hưởng của bias và hiệp phương sai; tái tích phân/cập nhật sau khi bias đổi; điều kiện khởi tạo attitude và gravity.
- **Tạo:** module preintegration với giao diện `integrate(sample, dt)`, `reset`, trạng thái `delta + covariance`, và quy tắc xử lý timestamp lỗi.
- **Kiểm chứng:** chuyển động đứng yên/quay đều/gia tốc đều khớp nghiệm tổng hợp; thêm bias đã biết làm sai số tăng và hiệu chỉnh bias kéo lại; covariance tăng hợp lý khi tích lũy lâu.
- **Nguồn:** [bài báo, III-B, (2)–(9)](https://arxiv.org/html/2007.00258), [imuPreintegration.cpp](https://github.com/TixiaoShan/LIO-SAM/blob/0be1fbe6275fb8366d5b800af4fc8c76a885c869/src/imuPreintegration.cpp).

### D2. Đồ thị IMU ngắn hạn và vòng phản hồi LiDAR

**Công thức cấu trúc [Suy ra từ factor trong source]:**

$$
\min_{\{R_i,p_i,v_i,b_i\}}
\sum_i\|r^{\rm imu}_{i,i+1}\|_{\Sigma_{\rm imu}}^2+
\sum_i\|b_{i+1}-b_i\|_{\Sigma_b}^2+
\sum_i\|\operatorname{Log}((T_i^{L})^{-1}T_i^{I})\|_{\Sigma_L}^2.
$$

$r^{\rm imu}=[r_R,r_v,r_p]$ dùng các residual vừa nêu ở D1; hạng giữa mô tả bias biến đổi chậm. Hạng cuối là **pose prior từ LiDAR correction** trong graph IMU của source, không phải phép cộng hai pose. $T_i^L$ phải đổi về đúng frame của trạng thái IMU trước khi tạo factor. Bài học cần chỉ ra lúc nào source cập nhật bias và reset preintegrator.

- **Nền:** prior, IMU factor, bias factor, correction từ LiDAR pose, dự đoán ở tần số IMU và tối ưu khi nhận pose LiDAR. Lý do reset đồ thị ngắn hạn nhưng giữ trạng thái cuối.
- **Thuật toán cần nắm:** luồng `IMU mẫu mới → dự đoán`; `LiDAR pose đã khớp → factor hiệu chỉnh → tối ưu pose/velocity/bias → reset preintegrator → dự đoán tiếp`. Source dùng pose LiDAR của các scan được mapping xử lý, không chỉ keyframe của đồ thị pose. Xem kỹ thời điểm đo và hai hàng đợi IMU của source.
- **Tạo:** đầu ra odometry nhanh và trạng thái bias, giao diện nhận LiDAR correction có timestamp/covariance, không tạo vòng feedback trùng lặp.
- **Kiểm chứng:** khi có bias gyro tổng hợp, odometry tự do trôi còn phiên bản có LiDAR correction giảm trôi; không mất tính liên tục khi reset đồ thị; phát hiện correction trễ hoặc ngoài khoảng.
- **Nguồn:** [README, hai đồ thị](https://github.com/TixiaoShan/LIO-SAM/blob/0be1fbe6275fb8366d5b800af4fc8c76a885c869/README.md), [imuPreintegration.cpp: odometryHandler, imuHandler, resetOptimization](https://github.com/TixiaoShan/LIO-SAM/blob/0be1fbe6275fb8366d5b800af4fc8c76a885c869/src/imuPreintegration.cpp).

### D3. Đồ thị pose keyframe dài hạn

**Công thức cấu trúc [Suy ra từ graph pose trong source]:**

$$
\min_{\{T_i\}}
\|r_{\rm prior}(T_0)\|_{\Sigma_0}^2+
\sum_{(i,j)\in\mathcal E_L}\|r^L_{ij}\|_{\Sigma_L}^2+
\sum_{(i,j)\in\mathcal E_{\rm loop}}\|r^{\rm loop}_{ij}\|_{\Sigma_{\rm loop}}^2+
\sum_{i\in\mathcal G}\|p(T_i)-p_i^{\rm GPS}\|_{\Sigma_{\rm GPS}}^2.
$$

Loop và GPS là các hạng **tùy chọn**. Chọn prior cho pose đầu để cố định gauge; sau khi nghiệm $T_i$ đổi, các cloud keyframe phải được chiếu lại theo pose mới. Công thức mô tả lớp factor; không tự nó quy định thư viện hoặc thời điểm gọi iSAM2.

- **Nền:** node pose, LiDAR odometry factor, prior đầu hành trình, iSAM2/incremental update, covariance và gauge freedom. Đồ thị này giữ lịch sử keyframe; đồ thị IMU D2 phục vụ nhịp nhanh.
- **Thuật toán cần nắm:** thêm factor LiDAR từ C4; cập nhật graph; nhận pose keyframe đã tối ưu; đồng bộ lại local-map cache khi pose lịch sử đổi; quan hệ odometry liên tục với pose/map đã sửa.
- **Tạo:** backend pose graph cho keyframes, đường xuất quỹ đạo đã tối ưu và tín hiệu “poses changed” cho map.
- **Kiểm chứng:** chuỗi factor thẳng cho quỹ đạo đúng; thêm một ràng buộc giữa hai pose làm nghiệm cập nhật như mong đợi; không đổi pose lịch sử mà giữ nguyên world-cloud cache cũ.
- **Nguồn:** [bài báo, III-A/III-C3](https://arxiv.org/html/2007.00258), [README, System architecture](https://github.com/TixiaoShan/LIO-SAM/blob/0be1fbe6275fb8366d5b800af4fc8c76a885c869/README.md), [mapOptmization.cpp: addOdomFactor, correctPoses](https://github.com/TixiaoShan/LIO-SAM/blob/0be1fbe6275fb8366d5b800af4fc8c76a885c869/src/mapOptmization.cpp).

## 7. Chặng E — ràng buộc dài hạn, sản phẩm bản đồ và đánh giá

### E1. Đóng vòng: ứng viên → kiểm chứng → factor

**Công thức kiểm tra và ràng buộc [Suy ra; Paper III-E]:** sau khi ICP xác nhận hai vùng trùng nhau, tạo $Z^{\rm loop}_{ij}$ và residual

$$
r^{\rm loop}_{ij}=\operatorname{Log}\!\left(
(Z^{\rm loop}_{ij})^{-1}T_i^{-1}T_j\right).
$$

ICP chỉ cung cấp một ứng viên đo hình học; cần kiểm tra fitness, số tương ứng và tính hợp lý trước khi đưa factor vào graph. Một $Z_{ij}$ sai có thể kéo cả quỹ đạo lệch.

- **Nền:** place recognition/ứng viên gần nhau theo pose, cách loại vòng lặp quá gần về thời gian, ICP, fitness, false loop và tác động tới toàn đồ thị.
- **Thuật toán cần nắm:** source gốc dùng tìm ứng viên theo khoảng cách rồi ICP với cloud keyframe lân cận; thêm factor khi kiểm tra đạt. Đóng vòng là **tùy chọn trong baseline**, không được giả định luôn xảy ra. Tách phát hiện ứng viên, xác nhận hình học và áp factor thành ba bước.
- **Tạo:** `LoopCandidate`, `LoopConstraint` và nhật ký chấp nhận/từ chối; khả năng bật/tắt loop để so sánh.
- **Kiểm chứng:** hai lần thấy cùng vùng tạo correction hợp lý; cặp giống hình học nhưng khác nơi không được chấp nhận; sau loop, quỹ đạo/map cập nhật và số keyframe vẫn nhất quán.
- **Nguồn:** [bài báo, III-E](https://arxiv.org/html/2007.00258), [README, Loop closure](https://github.com/TixiaoShan/LIO-SAM/blob/0be1fbe6275fb8366d5b800af4fc8c76a885c869/README.md), [mapOptmization.cpp: performLoopClosure, addLoopFactor](https://github.com/TixiaoShan/LIO-SAM/blob/0be1fbe6275fb8366d5b800af4fc8c76a885c869/src/mapOptmization.cpp).

### E2. GPS hoặc nguồn tuyệt đối khác (nhánh tùy chọn)

**Công thức factor vị trí [Suy ra từ Paper III-D]:**

$$
r_i^{\rm GPS}=p(T_i)-z_i^{\rm GPS},\qquad
E_i^{\rm GPS}=(r_i^{\rm GPS})^\top\Sigma_{{\rm GPS},i}^{-1}r_i^{\rm GPS}.
$$

$z_i^{\rm GPS}$ phải ở cùng hệ tọa độ và thời điểm với pose được ràng buộc. Residual này có ba thành phần vị trí; nó **không trực tiếp** đo hướng. Source ở commit đối chiếu chọn mẫu theo cửa sổ thời gian, nên phần đồng bộ phải có bài kiểm chứng riêng.

- **Nền:** vị trí toàn cục so với frame map cục bộ, hiệp phương sai, nội suy thời gian, ngoại chuẩn và hệ tọa độ địa lý → Cartesian.
- **Thuật toán cần nắm:** chính sách chấp nhận measurement theo chất lượng/covariance; factor vị trí không tự cung cấp roll/pitch/yaw; ảnh hưởng của GPS sai hoặc lệch thời gian. Bài báo nói nội suy GPS theo thời gian LiDAR; source tại commit đối chiếu lấy mẫu GPS trong cửa sổ thời gian và dùng nhiều điều kiện loại mẫu, không triển khai nội suy tuyến tính như mô tả ấy.
- **Tạo:** adapter GPS + factor tùy chọn, có thể tắt hoàn toàn mà đường LiDAR–IMU vẫn chạy.
- **Kiểm chứng:** GPS tốt giảm trôi trên quỹ đạo tổng hợp dài; GPS outlier không kéo hỏng quỹ đạo; tắt GPS tái tạo baseline không GPS.
- **Nguồn:** [bài báo, III-D](https://arxiv.org/html/2007.00258), [mapOptmization.cpp: addGPSFactor](https://github.com/TixiaoShan/LIO-SAM/blob/0be1fbe6275fb8366d5b800af4fc8c76a885c869/src/mapOptmization.cpp).

### E3. Bản đồ, quỹ đạo, lưu và phát lại

**Công thức tái dựng map [Suy ra]:**

$$
\mathcal M_{\rm global}=
\operatorname{Voxel}\!\left(
\bigcup_{j\in K}T_{W B_j}\big(\mathcal F_j^e\cup\mathcal F_j^p\big)
\right).
$$

Nếu feature lưu trong LiDAR frame thì dùng thêm $T_{BL}$. Sau loop, thay pose $T_{W B_j}$ bằng nghiệm mới rồi dựng lại; việc xếp chồng các cloud đã publish trước đó không tự dịch các điểm cũ.

- **Nền:** cloud feature raw trong keyframe, pose keyframe đã tối ưu, map dựng theo pose hiện tại; phân biệt cloud hiển thị tích lũy và map được dựng lại sau loop.
- **Thuật toán cần nắm:** cách tái tạo cloud toàn cục khi pose sửa, mức voxel khi xuất map, tính nhất quán giữa trajectory và map, định dạng lưu chứa thông tin để phát lại hoặc kiểm tra.
- **Tạo:** đường xuất quỹ đạo, map cloud, metadata cấu hình/source/dataset; giao diện truy vấn keyframe và phiên bản pose. Định dạng lưu cụ thể là quyết định thiết kế riêng theo `docs/WORKFLOW.md`.
- **Kiểm chứng:** xuất lại map sau loop cho cloud thay đổi theo pose đã sửa; phát lại cùng dữ liệu/cấu hình cho cùng số keyframe và quỹ đạo trong sai số định trước.
- **Nguồn:** [README, Map (cloud) so với Map (global)](https://github.com/TixiaoShan/LIO-SAM/blob/0be1fbe6275fb8366d5b800af4fc8c76a885c869/README.md), [mapOptmization.cpp: correctPoses, publishFrames, saveMapService](https://github.com/TixiaoShan/LIO-SAM/blob/0be1fbe6275fb8366d5b800af4fc8c76a885c869/src/mapOptmization.cpp).

### E4. Tích hợp ROS và đánh giá trên dữ liệu thật

**Công thức chỉ số [Suy ra; phải ghi cách căn chỉnh]:** sau khi chọn phép căn chỉnh $S$ giữa quỹ đạo ước lượng $\hat T_i$ và ground truth $T_i$,

$$
\mathrm{ATE}_{\rm RMSE}
=\sqrt{\frac1N\sum_i\|\operatorname{trans}(T_i)-\operatorname{trans}(S\hat T_i)\|^2},
$$

$$
E^{\rm RPE}_{i,j}
=(T_i^{-1}T_j)^{-1}(\hat T_i^{-1}\hat T_j).
$$

Với RPE, báo riêng độ lớn tịnh tiến và góc quay của $E^{\rm RPE}$, kèm cách chọn cặp $(i,j)$ theo thời gian/quãng đường. Không so hai số ATE nếu phép căn chỉnh, đơn vị hoặc đoạn quỹ đạo khác nhau. Thời gian chạy mỗi scan cần so với chu kỳ scan và báo phân vị, không chỉ trung bình.

- **Nền:** adapter ROS ↔ dữ liệu thuật toán, QoS, clock mô phỏng, TF, hàng đợi, luồng xử lý, backpressure, thông số cấu hình, reproducibility. Lõi thuật toán nên được kiểm chứng bằng dữ liệu độc lập với ROS; cấu trúc package cụ thể sẽ quyết định khi bắt đầu code.
- **Đường kiểm chứng:** (1) dữ liệu tổng hợp có ground truth; (2) bag mẫu hợp lệ của tác giả chạy source gốc và bản triển khai trên cùng cấu hình; (3) dữ liệu dự án khi hợp đồng đầu vào A3 đạt; (4) cảnh khó: ít đặc trưng, quay nhanh, IMU nhiễu, mất gói, loop sai.
- **Chỉ số:** ATE/RPE kèm quy tắc căn chỉnh và đơn vị, độ phẳng/chồng khít map, tỷ lệ scan hợp lệ/keyframe, số tương ứng, tỷ lệ suy biến, độ trễ p50/p95/p99 mỗi module, CPU/RAM và tốc độ chạy so với tốc độ dữ liệu. Báo cả kết quả trước và sau loop/GPS.
- **Điều kiện so sánh:** cùng dữ liệu, tham số được lưu, version code, seed nếu có, phần cứng và cách đo thời gian. Dùng một baseline đo được trước khi thay thuật toán; thay đổi một module mỗi lượt để biết nguyên nhân.
- **Nguồn:** [bài báo, IV Experiments](https://arxiv.org/html/2007.00258), [README, Sample datasets](https://github.com/TixiaoShan/LIO-SAM/blob/0be1fbe6275fb8366d5b800af4fc8c76a885c869/README.md), [quy trình dự án](../WORKFLOW.md).

## 8. Các mốc hoàn thành và quyết định còn mở

### Những chỗ phải đọc cả bài báo lẫn source trước khi code

| Chủ đề | Bài báo trình bày | Source tại commit đối chiếu | Hệ quả cho bài học |
| --- | --- | --- | --- |
| Kiến trúc factor | Một sơ đồ chung với IMU, LiDAR, GPS, loop | Hai graph: pose keyframe dài hạn và trạng thái IMU ngắn hạn; correction LiDAR là pose prior trong graph IMU | Vẽ cả sơ đồ ý tưởng và hai đường dữ liệu thật. [README](https://github.com/TixiaoShan/LIO-SAM/blob/0be1fbe6275fb8366d5b800af4fc8c76a885c869/README.md), [imuPreintegration.cpp](https://github.com/TixiaoShan/LIO-SAM/blob/0be1fbe6275fb8366d5b800af4fc8c76a885c869/src/imuPreintegration.cpp#L350-L395) |
| Deskew | Mô tả ước lượng chuyển động trong scan bằng IMU | `imuDeskewInfo()` tích lũy gyro; `findPosition()` để tịnh tiến bằng 0 | Không gộp gyro deskew với GTSAM preintegration factor. [imageProjection.cpp](https://github.com/TixiaoShan/LIO-SAM/blob/0be1fbe6275fb8366d5b800af4fc8c76a885c869/src/imageProjection.cpp#L305-L362), [findPosition()](https://github.com/TixiaoShan/LIO-SAM/blob/0be1fbe6275fb8366d5b800af4fc8c76a885c869/src/imageProjection.cpp#L473-L510) |
| Local map | Cửa sổ sub-keyframe gần đây với số lượng cố định | Chọn theo bán kính quanh pose, lấy mẫu pose, thêm keyframe mới; feature cloud được voxel hóa | Không mặc định mã dùng đúng số keyframe trong paper. [Bài báo, III-C1](https://arxiv.org/html/2007.00258), [extractNearby()](https://github.com/TixiaoShan/LIO-SAM/blob/0be1fbe6275fb8366d5b800af4fc8c76a885c869/src/mapOptmization.cpp#L862-L938) |
| Ngưỡng keyframe | Thí nghiệm nêu 1 m / 10° | Cấu hình đối chiếu là 1 m / 0,2 rad và `saveFrame()` kiểm tra các góc | Đọc tham số, đổi đúng đơn vị; ghi version tham số khi so sánh. [params.yaml](https://github.com/TixiaoShan/LIO-SAM/blob/0be1fbe6275fb8366d5b800af4fc8c76a885c869/config/params.yaml#L75-L79), [saveFrame()](https://github.com/TixiaoShan/LIO-SAM/blob/0be1fbe6275fb8366d5b800af4fc8c76a885c869/src/mapOptmization.cpp#L1354-L1379) |
| Tối ưu LiDAR | Diễn đạt bằng tối ưu pose và residual | Hàm tên `LMOptimization()` giải normal equations và chiếu bỏ hướng suy biến; tên gọi không chứng minh có LM damping | Đọc phép tính trong hàm trước khi chọn solver tương đương. [LMOptimization()](https://github.com/TixiaoShan/LIO-SAM/blob/0be1fbe6275fb8366d5b800af4fc8c76a885c869/src/mapOptmization.cpp#L1158-L1279) |
| GPS | Paper mô tả nội suy theo timestamp | Code lấy mẫu trong cửa sổ ±0,2 s rồi qua nhiều cổng chất lượng | Kiểm tra riêng đồng bộ GPS của implementation. [Bài báo, III-D](https://arxiv.org/html/2007.00258), [addGPSFactor()](https://github.com/TixiaoShan/LIO-SAM/blob/0be1fbe6275fb8366d5b800af4fc8c76a885c869/src/mapOptmization.cpp#L1397-L1475) |
| Loop | Khả năng tích hợp bộ nhận diện nơi chốn khác | Ví dụ code dùng ứng viên theo khoảng cách/thời gian và ICP | Không mô tả Scan Context như thành phần có sẵn của baseline. [README, Loop closure](https://github.com/TixiaoShan/LIO-SAM/blob/0be1fbe6275fb8366d5b800af4fc8c76a885c869/README.md), [performLoopClosure()](https://github.com/TixiaoShan/LIO-SAM/blob/0be1fbe6275fb8366d5b800af4fc8c76a885c869/src/mapOptmization.cpp#L529-L643) |

| Mốc | Bắt buộc thấy được | Chưa cần ở mốc này |
| --- | --- | --- |
| M1 — dữ liệu và hình học | A0–A3; frame/time/extrinsic có bài kiểm chứng độc lập | scan matching |
| M2 — frontend | B1–B4; deskew và đặc trưng có dữ liệu tổng hợp kiểm chứng | đồ thị tối ưu |
| M3 — LiDAR odometry | C1–C4; local map, pose scan-to-map, keyframe và factor LiDAR | fusion IMU hoàn chỉnh |
| M4 — LIO-SAM cốt lõi | D1–D3; IMU prediction, LiDAR correction, hai đồ thị và quỹ đạo | GPS/loop |
| M5 — SLAM dài hạn | E1 + E3; đóng vòng được kiểm chứng và map được dựng lại | GPS nếu không có cảm biến |
| M6 — sẵn sàng so sánh thiết kế | E4; có baseline định lượng, cảnh lỗi và tài liệu tái lập | kết luận thuật toán “tối ưu nhất” |

Các quyết định **chưa chốt trong dàn ý**: quy ước frame chi tiết cho implementation mới; nguồn và kiểu dữ liệu LiDAR có `time/ring`; cách xử lý scan thiếu dữ liệu; dùng thư viện tối ưu nào; mức trung thành với lựa chọn mặc định của source; biểu diễn map và định dạng lưu; có dùng GPS không; tiêu chí chấp nhận loop; bộ dữ liệu chuẩn và ngưỡng hiệu năng. Khi chốt một lựa chọn ảnh hưởng giao diện hoặc thuật toán, ghi [quyết định thiết kế](../WORKFLOW.md) và phép đo đối chiếu.

### Danh mục source gốc nên giữ cố định khi học

- [LIO-SAM paper, arXiv:2007.00258v3, 2020](https://arxiv.org/html/2007.00258): định nghĩa, công thức và thí nghiệm.
- [README tại commit 0be1fbe](https://github.com/TixiaoShan/LIO-SAM/blob/0be1fbe6275fb8366d5b800af4fc8c76a885c869/README.md): kiến trúc hai đồ thị và yêu cầu cảm biến.
- [imageProjection.cpp](https://github.com/TixiaoShan/LIO-SAM/blob/0be1fbe6275fb8366d5b800af4fc8c76a885c869/src/imageProjection.cpp), [featureExtraction.cpp](https://github.com/TixiaoShan/LIO-SAM/blob/0be1fbe6275fb8366d5b800af4fc8c76a885c869/src/featureExtraction.cpp), [imuPreintegration.cpp](https://github.com/TixiaoShan/LIO-SAM/blob/0be1fbe6275fb8366d5b800af4fc8c76a885c869/src/imuPreintegration.cpp), [mapOptmization.cpp](https://github.com/TixiaoShan/LIO-SAM/blob/0be1fbe6275fb8366d5b800af4fc8c76a885c869/src/mapOptmization.cpp): hành vi triển khai. Tên `mapOptmization.cpp` là cách viết trong source gốc.
- [cloud_info.msg](https://github.com/TixiaoShan/LIO-SAM/blob/0be1fbe6275fb8366d5b800af4fc8c76a885c869/msg/cloud_info.msg) và [params.yaml](https://github.com/TixiaoShan/LIO-SAM/blob/0be1fbe6275fb8366d5b800af4fc8c76a885c869/config/params.yaml): hợp đồng dữ liệu và cấu hình của bản gốc.

**Thứ tự làm ngay:** 0.1 → A0 → A1 → A2 → A3. Chỉ sau khi đọc, làm bài tính tay và kiểm chứng được các mục này mới mở B1. Mỗi lần chỉ triển khai một module nhỏ với điều kiện hoàn thành nêu trên; dàn ý này không tự động duyệt thay các quyết định thiết kế còn mở.
