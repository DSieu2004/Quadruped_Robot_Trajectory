# PID controller library

`quadruped::pid` là thư viện C++17 độc lập, không phụ thuộc ROS hay MuJoCo.
Mỗi `pid::PidController` điều khiển một trục; tạo một instance riêng cho mỗi
khớp hoặc trục thân.

```cpp
#include "pid/pid_controller.hpp"

pid::PidGains gains;
gains.kp = 8.0;
gains.ki = 1.5;
gains.kd = 4.0;
gains.output_min = -30.0;
gains.output_max = 30.0;
pid::PidController controller(gains);

double torque = controller.update(q_reference, q_measured, dt);
```

Luật rời rạc:

```text
e = setpoint - measurement
P = Kp * e
I = clamp(I + Ki * trapezoid(e) * dt, I_min, I_max)
D = -Kd * low_pass(d(measurement) / dt)
u = feedforward + P + I + D
output = clamp(u, output_min, output_max)
```

Đạo hàm trên measurement tránh derivative kick khi đổi setpoint. Bộ lọc thông thấp
giảm nhiễu; back-calculation anti-windup hạn chế tích phân khi output bão hòa.
`dt` phải nằm trong `(0, max_dt]`. Đơn vị tùy hệ điều khiển: với khớp là radian,
giây và N·m.

`reset()` xóa lịch sử và đặt I-term. Overload
`reset(setpoint, measurement, desired_output, feedforward)` khởi tạo gần như
bumpless trong giới hạn cấu hình. `state()` cung cấp P/I/D, output trước/sau
saturation và cờ bão hòa để ghi log.

Ví dụ độc lập nằm ở [pid_channel.cpp](../../examples/pid_channel.cpp). PID library
đã được build và export; Go2 adapter hiện vẫn dùng SMC. Việc chọn PID trong Go2
là bước tích hợp tiếp theo.
