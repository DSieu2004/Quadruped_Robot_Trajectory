# Báo cáo tạo và bám quỹ đạo Go2

## 1. Mục tiêu và phạm vi

Báo cáo mô tả bộ tạo quỹ đạo có timestamp và ba luật bám quỹ đạo PID, SMC, MPC trong mô phỏng MuJoCo/ROS 2. Phạm vi ở đây là **bộ điều khiển bám quỹ đạo cấp cao**. Trong các lượt PID/SMC đã ghi và MPC mới tích hợp, bridge Go2 vẫn dùng SMC low-level để giữ cân bằng thân và điều khiển chân; do đó phép so sánh chỉ thay đổi bộ bám trajectory.

Khi chạy trajectory, follower phát trực tiếp `/cmd_vel`, không gọi planner, costmap hay local controller của Nav2. Launch cũng tắt các vật cản/vách tĩnh của scene, nhưng giữ sàn phẳng. Vị trí thực lấy từ `/odom`.

## 2. Luồng xử lý

1. Follower đợi odometry đầu tiên và chụp pose bắt đầu `(x_0, y_0, ψ_0)`.
2. Sau thời gian chờ, nó tạo mẫu tham chiếu theo thời gian mô phỏng. Mỗi pose tham chiếu được công bố trên `trajectory/reference` với frame `odom` và timestamp riêng.
3. Ở mỗi chu kỳ điều khiển, follower đọc pose/vận tốc thực, tính lệnh từ PID, SMC hoặc MPC theo `control_law`.
4. Lệnh vận tốc được giới hạn, đổi từ hệ tọa độ thế giới sang hệ thân robot rồi phát trên `/cmd_vel`.
5. Follower lưu telemetry theo từng mẫu vào CSV; plotter hiển thị quỹ đạo, đáp ứng vị trí, sai số và lệnh vận tốc.

Mỗi lượt bắt đầu tại vị trí ban đầu của robot. Quỹ đạo cục bộ `(x_r, y_r)` được quay theo yaw ban đầu rồi tịnh tiến về `(x_0,y_0)`:

```text
x_ref = x_0 + cos(ψ_0) x_r - sin(ψ_0) y_r
y_ref = y_0 + sin(ψ_0) x_r + cos(ψ_0) y_r
```

Vận tốc và gia tốc tham chiếu cũng được quay bằng cùng phép biến đổi. Thời gian chạy của đường là `period × cycles`; follower ghi thêm tối đa 2 giây sau khi hoàn thành để quan sát robot dừng/giữ vị trí.

## 3. Sinh quỹ đạo có timestamp

Đặt `ω = 2π/T`, trong đó `T` là `period` (giây cho một vòng) và `t` là thời gian từ lúc bắt đầu trajectory. Các hàm dưới đây định nghĩa vị trí cục bộ; vận tốc và gia tốc tham chiếu là đạo hàm theo thời gian.

### 3.1 Circle

Với bán kính `r`:

```text
x_r(t) = r sin(ωt)              y_r(t) = r [1 - cos(ωt)]
vx_r(t) = rω cos(ωt)            vy_r(t) = rω sin(ωt)
ax_r(t) = -rω² sin(ωt)          ay_r(t) = rω² cos(ωt)
```

Quỹ đạo bắt đầu tại gốc cục bộ `(0,0)` và tiếp tuyến ban đầu hướng theo trục `+x`.

### 3.2 Ellipse

Với bán trục `a` theo `x` và `b` theo `y`:

```text
x_r(t) = a sin(ωt)              y_r(t) = b [1 - cos(ωt)]
vx_r(t) = aω cos(ωt)            vy_r(t) = bω sin(ωt)
ax_r(t) = -aω² sin(ωt)          ay_r(t) = bω² cos(ωt)
```

### 3.3 Lemniscate (hình số 8)

Với các hệ số kích thước `a`, `b`:

```text
x_r(t) = a sin(ωt)              y_r(t) = (b/2) sin(2ωt)
vx_r(t) = aω cos(ωt)            vy_r(t) = bω cos(2ωt)
ax_r(t) = -aω² sin(ωt)          ay_r(t) = -2bω² sin(2ωt)
```

Hướng yaw tham chiếu được đặt theo tiếp tuyến:

```text
ψ_ref = atan2(vy_ref, vx_ref)
ω_ref = (vx_ref ay_ref - vy_ref ax_ref) / (vx_ref² + vy_ref²)
```

`ω_ref` được đặt bằng 0 khi bình phương tốc độ gần 0 để tránh chia cho số quá nhỏ. Mẫu của `nav_msgs/Path` cách nhau 0.1 giây và có `header.stamp = start_time + t`; vòng bám chạy bằng timer 50 ms. Giới hạn mặc định của lệnh là 0.12 m/s và 0.18 rad/s. Follower từ chối cấu hình mà tốc độ cực đại tham chiếu vượt 90% giới hạn tốc độ, để chừa biên cho bộ điều khiển sửa sai số.

## 4. PID bám quỹ đạo

PID được giải thích trước vì đây là luật phản hồi trực tiếp theo sai số vị trí. Bộ follower có ba bộ PID độc lập: trục `x`, trục `y` trong hệ thế giới và yaw. Các trục `x/y` nhận vị trí tham chiếu/thực; yaw nhận sai số góc đã wrap về `[-π,π]`. Vận tốc tham chiếu được đưa vào như feedforward.

Với một trục vị trí, đặt:

```text
e = p_ref - p
P = Kp e
I* = clamp(I_prev + Ki (e_prev + e) dt / 2, I_min, I_max)
u* = v_ref + P + I* - Kd d_f
u  = clamp(u*, -v_max, v_max)
```

Đạo hàm dùng tốc độ đo (derivative on measurement), không lấy đạo hàm trực tiếp trên setpoint. Tốc độ đo được lọc thông thấp bậc nhất:

```text
d_raw = (p - p_prev) / dt
α = exp(-dt/τ)
d_f = α d_f_prev + (1 - α) d_raw
```

Cách này giảm derivative kick khi tham chiếu thay đổi. Sau khi giới hạn output, tích phân được hiệu chỉnh anti-windup bằng back-calculation:

```text
I = clamp(I* + (u - u*) [1 - exp(-K_aw dt)], I_min, I_max)
```

Sau hiệu chỉnh, output được tính lại và giới hạn lần nữa. Nhờ đó tích phân không tiếp tục tích lũy mạnh khi lệnh vận tốc chạm ngưỡng. Với yaw, sai số đầu vào là `e_ψ = wrap(ψ_ref - ψ)` và feedforward là `ω_ref`; output là tốc độ quay yaw đã giới hạn.

Gain mặc định trong trajectory follower:

| Kênh | `Kp` | `Ki` | `Kd` | Lọc đạo hàm `τ` | Giới hạn tích phân |
|---|---:|---:|---:|---:|---:|
| `x`, `y` | 1.6 | 0.04 | 0.18 | 0.15 s | ±0.035 |
| yaw | 1.5 | 0.02 | 0.20 | 0.12 s | ±0.04 |

`K_aw = 5`, giới hạn `dt` của PID là 0.25 s. Output vị trí là vận tốc (m/s); output yaw là tốc độ góc (rad/s). Vector vận tốc tổng thể tiếp tục được giới hạn theo `max_speed`, rồi đổi sang hệ thân robot trước khi phát `/cmd_vel`.

## 5. SMC bám quỹ đạo

SMC điều khiển sai số vị trí và sai số vận tốc theo một mặt trượt. Với quy ước trong code:

```text
e = p - p_ref
ė = v - v_ref
s = ė + λe
```

Đầu ra gia tốc ảo cho từng trục là:

```text
a_cmd = a_ref - λė - k_l s - k_s sat(s / φ)
sat(z) = clamp(z, -1, 1)
a_out = clamp(a_cmd, -a_max, a_max)
```

`λ` đặt tốc độ hội tụ của mặt trượt; `k_l` là thành phần tuyến tính; `k_s` là độ mạnh thành phần chuyển mạch; `φ` là độ dày boundary layer. Hàm `sat` thay cho sign lý tưởng trong vùng biên để giảm rung điều khiển. `a_ref` tạo feedforward theo gia tốc quỹ đạo.

Vì interface cuối là vận tốc, follower tích phân gia tốc ảo một bước:

```text
v_cmd_world = v_actual_world + dt a_out
```

Lệnh vận tốc SMC qua bộ lọc thông thấp bậc nhất để giảm gợn vận tốc do gait/odometry trước khi giới hạn tốc độ và đổi sang hệ thân robot:

```text
v_f[k] = α v_f[k-1] + (1 - α) v_cmd[k]
α = exp(-dt/τ_v)
```

`τ_v=0.25 s` cho vận tốc tuyến tính. Yaw dùng một luật reaching riêng với sai số `wrap(ψ - ψ_ref)` và sai số tốc độ góc `ω - ω_ref`; gia tốc yaw được tích phân thành lệnh tốc độ quay rồi lọc tương tự với `τ_ω=0.30 s` và giới hạn bởi `max_yaw_rate`. PID không đi qua các bộ lọc SMC này.

Thông số SMC trajectory hiện tại đã được làm mềm để giảm chuyển mạch: vị trí `λ=2.0`, `k_l=1.5`, `k_s=0.04`, `φ=0.45`, `a_max=0.15 m/s²`; yaw `λ=3.0`, `k_l=2.5`, `k_s=0.08`, `φ=0.18`, giới hạn gia tốc góc `0.35 rad/s²`. Giới hạn gia tốc 0.15 m/s² đồng thời được chuyển cho bridge làm giới hạn ramp lệnh; mức 0.22 m/s² từng làm robot ngã trong lượt thử lemniscate, nên không tăng giới hạn này.

## 5.1. MPC bám quỹ đạo

MPC dùng mô hình double-integrator rời rạc cho từng trục tịnh tiến và yaw:

```text
p[k+1] = p[k] + dt v[k] + 0.5 dt² a[k]
v[k+1] = v[k] + dt a[k]
```

Tại mỗi chu kỳ, controller lấy `N` mẫu tham chiếu tương lai và tối ưu chuỗi gia tốc `a[0...N−1]` theo hàm mục tiêu:

```text
J = Σ (Qp (p[k] - p_ref[k])²
      + Qv (v[k] - v_ref[k])²
      + R a[k]²
      + S (a[k] - a[k-1])²)
```

Gia tốc mỗi bước bị chặn trong `[-a_max, a_max]`, đồng thời giới hạn `|a[k]-a[k-1]| ≤ Δa_max`. Thành phần `R` tránh điều khiển quá mạnh; `S` và `Δa_max` làm lệnh thay đổi từ từ thay vì nhảy qua hai cực trị. Bài toán quadratic được khai triển thành QP có ràng buộc gia tốc và slew rồi giải bằng projected-gradient; nghiệm được warm-start bằng cách dịch chuỗi điều khiển của chu kỳ trước. Chỉ `a[0]` được áp dụng, sau đó MPC dự đoán và tối ưu lại ở chu kỳ tiếp theo.

Mặc định dùng horizon 15 bước, trọng số vị trí/vận tốc/gia tốc/đổi gia tốc `20/3/0.5/2.0`; yaw dùng `8/2/3/2.0`. Đổi gia tốc tối đa mỗi chu kỳ là `0.04 m/s²` cho x/y và `0.06 rad/s²` cho yaw. Sau khi tích phân gia tốc thành vận tốc, đầu ra MPC được lọc với `τ=0.25 s` (yaw `0.30 s`) để giảm ripple vận tốc đo đi vào lệnh. Giới hạn gia tốc dùng chung cấu hình trajectory (`0.15 m/s²` cho x/y và `0.35 rad/s²` cho yaw). Đây là MPC cấp trajectory; bridge thân/chân vẫn dùng controller low-level riêng.

## 6. Ghi dữ liệu và đánh giá

Mỗi CSV ghi các trường:

```text
sim_time, reference_x, reference_y, reference_yaw,
actual_x, actual_y, actual_yaw,
cmd_vx, cmd_vy, cmd_yaw_rate,
controller_x_output, controller_y_output, controller_yaw_output,
trajectory_law
```

Các cột `controller_*_output` phụ thuộc luật đang chạy: với SMC/MPC chúng là gia tốc ảo; với PID chúng là output vận tốc/tốc độ yaw của PID. Vì vậy không nên gộp chúng vào cùng một biểu đồ đại lượng vật lý mà không đổi nhãn/đơn vị. CSV hiện hữu được lưu dưới `logs/` và script so sánh đọc vị trí tham chiếu/thực để tính:

```text
e_x = x_ref - x_actual
e_y = y_ref - y_actual
e_xy = sqrt(e_x² + e_y²)
RMS_xy = sqrt(mean(e_xy²))
max_xy = max(e_xy)
```

Đồ thị so sánh chỉ hiển thị các đại lượng trực tiếp hữu ích: đường XY tham chiếu và ba đường robot, độ lớn sai số theo thời gian, `e_x` và `e_y`. Mốc sau thời lượng đường cộng thêm 2 giây dừng không tính vào thống kê. Biểu đồ mới: [circle PID/SMC/MPC](../logs/trajectory_circle_pid_vs_smc_vs_mpc.png), [lemniscate PID/SMC/MPC](../logs/trajectory_lemniscate_pid_vs_smc_vs_mpc.png).

## 7. Hình ảnh mô phỏng

Chèn screenshot MuJoCo/RViz vào các vị trí dưới đây. Khi ảnh đã được lưu trong workspace, thay dòng hướng dẫn bằng cú pháp Markdown `![mô tả ảnh](đường_dẫn_tương_đối_từ_docs/)`.

### 7.1 Circle

**Vị trí chèn ảnh:** Robot đang chạy quỹ đạo circle trong MuJoCo.

<!-- Ví dụ: ![Go2 bám quỹ đạo circle](../images/trajectory/circle.png) -->

*Hình 1. Go2 bám quỹ đạo circle trong mô phỏng.*

### 7.2 Lemniscate

**Vị trí chèn ảnh:** Robot đang chạy quỹ đạo lemniscate (hình số 8) trong MuJoCo.

<!-- Ví dụ: ![Go2 bám quỹ đạo lemniscate](../images/trajectory/lemniscate.png) -->

*Hình 2. Go2 bám quỹ đạo lemniscate trong mô phỏng.*

### 7.3 So sánh PID, SMC và MPC

**Vị trí chèn ảnh:** Screenshot mô phỏng hoặc cửa sổ đồ thị thể hiện đường chạy và sai số của PID/SMC/MPC.

<!-- Ví dụ: ![So sánh PID, SMC và MPC](../images/trajectory/pid_smc_mpc.png) -->

*Hình 3. Kết quả mô phỏng và đáp ứng bám quỹ đạo của PID/SMC/MPC.*

## 8. Kết quả mô phỏng đã ghi

Các lượt dưới đây dùng cùng hình học cho mỗi dạng đường, scene phẳng đã tắt vật cản tĩnh và bridge Go2 SMC low-level. Lưu ý các CSV MPC trong bảng được tạo trước khi MPC được nối vào tầng low-level; chúng là kết quả trajectory MPC + low-level SMC. Full-MPC low-level đã được tích hợp thử nhưng hiện chưa ổn định khi contact chuyển pha; chưa dùng nó để tạo benchmark. Nguyên nhân là MPC theo từng kênh chưa mô hình hóa ràng buộc tiếp xúc và phân bố lực toàn thân trong horizon. RMS là căn trung bình bình phương độ lớn sai số XY; `max` là sai số XY lớn nhất trên thời lượng quỹ đạo. Dữ liệu PID/SMC/MPC trong bảng chỉ so sánh trajectory law trên cùng low-level SMC.

| Quỹ đạo | Luật bám | RMS XY | Max XY | Sai số cuối | CSV |
|---|---|---:|---:|---:|---|
| Circle, `r=1.5 m`, `T=100 s` | SMC | 0.84 cm | 5.33 cm | 0.47 cm | [CSV SMC](../logs/trajectory_circle_smc_20261026_112708.csv) |
| Circle, `r=1.5 m`, `T=100 s` | PID | 0.96 cm | 1.41 cm | 1.04 cm | [CSV PID](../logs/trajectory_circle_pid_20261026_105716.csv) |
| Circle, `r=1.5 m`, `T=100 s` | MPC | 1.08 cm | 6.71 cm | 0.58 cm | [CSV MPC](../logs/trajectory_circle_mpc_20261026_114905.csv) |
| Lemniscate, `a=1.2 m`, `b=0.75 m`, `T=120 s` | SMC | 0.63 cm | 3.54 cm | 0.07 cm | [CSV SMC](../logs/trajectory_lemniscate_smc_20261026_113234.csv) |
| Lemniscate, `a=1.2 m`, `b=0.75 m`, `T=120 s` | PID | 0.57 cm | 1.94 cm | 0.71 cm | [CSV PID](../logs/trajectory_lemniscate_pid_20261026_110441.csv) |
| Lemniscate, `a=1.2 m`, `b=0.75 m`, `T=120 s` | MPC | 0.66 cm | 3.45 cm | 0.13 cm | [CSV MPC](../logs/trajectory_lemniscate_mpc_20261026_115007.csv) |

Trajectory MPC đã hoàn thành cả hai đường; với lemniscate, RMS của nó (0.66 cm) gần SMC (0.63 cm) và PID (0.57 cm). Trên circle, RMS là 1.08 cm; sai số lớn nhất 6.71 cm chủ yếu do transient đầu lượt. Đây là kết quả trajectory MPC với low-level SMC, không đại diện cho full-MPC. Full-MPC hiện ngã trong các lần thử low-level, nên chưa có benchmark full-stack đáng tin cậy.

## 9. Chạy mô phỏng và tạo đồ thị so sánh

Ví dụ chạy lemniscate bằng PID:

```bash
cd /home/dinhsieu/quadruped_ws
bash scripts/run_trajectory.bash shape:=lemniscate control_law:=pid
```

Chạy MPC với cùng cấu hình circle/lemniscate đã dùng trong các lượt PID/SMC:

```bash
bash scripts/run_trajectory.bash shape:=circle control_law:=mpc radius:=1.5 period:=100 cycles:=1
bash scripts/run_trajectory.bash shape:=lemniscate control_law:=mpc semi_axis_x:=1.2 semi_axis_y:=0.75 period:=120 cycles:=1
```

Đổi `control_law:=smc`, `control_law:=pid` hoặc `control_law:=mpc` để chạy từng bộ bám; circle dùng `shape:=circle`. Script tự ghi CSV theo tên chứa shape, luật điều khiển và thời điểm chạy. Sau khi đã có log của cả ba luật cho cùng quỹ đạo, tạo biểu đồ so sánh:

```bash
/usr/bin/python src/trajectory/trajectory_compare.py --shape circle
/usr/bin/python src/trajectory/trajectory_compare.py --shape lemniscate
```

Script in thời lượng và bảng RMS/max/final, đồng thời lưu PNG trong `logs/`. Có thể truyền thủ công `--smc smc.csv --pid pid.csv --mpc mpc.csv` nếu muốn so sánh các lượt cụ thể thay vì tự chọn log mới nhất.

## 10. Mã nguồn liên quan

- [trajectory_follower.cpp](../src/trajectory/trajectory_follower.cpp): sinh timestamp, đọc odometry, chọn luật PID/SMC, phát `/cmd_vel` và ghi CSV.
- [trajectory_compare.py](../src/trajectory/trajectory_compare.py): đọc log, tính sai số và tạo biểu đồ/bảng so sánh.
- [trajectory_plotter.py](../src/trajectory/trajectory_plotter.py): đồ thị telemetry trực tiếp trong lúc mô phỏng.
- [trajectory.launch.xml](../ros2/go2_smc_ros/launch/trajectory.launch.xml): tham số mô phỏng và node trajectory.
- [run_trajectory.bash](../scripts/run_trajectory.bash): wrapper khởi chạy và đặt tên file CSV.
- [pid_controller.cpp](../libraries/pid/src/pid_controller.cpp) và [reaching_law.cpp](../libraries/smc/src/reaching_law.cpp): triển khai luật rời rạc PID và SMC.
- [mpc_controller.hpp](../libraries/mpc/include/mpc/mpc_controller.hpp) và [mpc_controller.cpp](../libraries/mpc/src/mpc_controller.cpp): API và bộ giải MPC double-integrator.
