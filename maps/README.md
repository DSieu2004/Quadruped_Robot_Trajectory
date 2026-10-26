# Map phòng lớn Go2 tạo bằng SLAM

Map được tạo ngày 2026-10-24 bằng [bài thử toàn luồng](../scripts/test_slam_nav.bash), không vẽ từ world XML.

- Robot: Unitree Go2 Menagerie, điều khiển mô-men bằng SMC C++.
- World: [slam_scene.xml](../models/quadruped/slam_scene.xml), phòng 10 × 8 m có tường, hai cột, hai vách ngăn và một khối hộp.
- Đầu vào: LiDAR ray-casting 720 tia, 10 Hz, tầm 12 m; odometry lý tưởng từ MuJoCo.
- Thuật toán: SLAM Toolbox Humble / Ceres, loop closing bật.
- Occupancy grid: [go2_slam.yaml](go2_slam.yaml), [go2_slam.pgm](go2_slam.pgm), **197 × 157 cell**, 0.05 m/cell, phủ khoảng 9.85 × 7.85 m.
- Trạng thái SLAM: `go2_slam.posegraph` và `go2_slam.data`, xuất bằng service serialize_map.

Map này đã được nạp lại bằng map server + AMCL. Nav2 A* và plugin SMC đã hoàn thành ba mục tiêu xa nhau trong phòng:

| Mục tiêu map | Sai số feedback cuối | Độ lệch ngang lớn nhất trên tuyến | Tiếp xúc vật cản / ngã |
| --- | ---: | ---: | --- |
| `(4.6, 0.0)` | 0.0664 m | 0.9727 m | 0 / 0 |
| `(-2.5, 2.5)` | 0.0652 m | 2.4874 m | 0 / 0 |
| `(0.0, 0.0)` quay về | 0.0993 m | 2.4950 m | 0 / 0 |

Các số vị trí trên được ghi lúc tạo map với controller phiên bản trước. Bản điều khiển hiện tại hỗ trợ quay theo đường và yaw cuối của goal, dùng tolerance vị trí 0.12 m và yaw 0.08 rad. Kiểm thử heading riêng nằm trong [test_heading_nav.bash](../scripts/test_heading_nav.bash); CSV chi tiết được ghi vào `logs/`. Sai số lấy từ feedback `NavigateToPose`, không phải sai số của cảm biến robot thật. Xem [hướng dẫn SLAM/Nav2](../docs/SLAM_NAV2.md).
