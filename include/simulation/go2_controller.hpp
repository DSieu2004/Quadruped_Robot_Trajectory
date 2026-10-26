#pragma once

#include "locomotion/force_allocator.hpp"
#include "navigation/waypoint_navigator.hpp"
#include "pid/pid_controller.hpp"
#include "mpc/mpc_controller.hpp"
#include "smc/reaching_law.hpp"

#include <mujoco/mujoco.h>
#include <array>
#include <vector>

namespace simulation {
enum class Mode { Passive, Stand, Navigate, Velocity };
enum class ControlLaw { Smc, Pid, Mpc };
const char* mode_name(Mode mode);
const char* control_law_name(ControlLaw law);

struct Go2Config {
  double body_height = 0.31;
  double gait_period = 0.65;
  double stance_fraction = 0.65;
  double swing_clearance = 0.065;
  double settle_time = 1.5;
  navigation::NavigatorConfig navigation;
  smc::ReachingGains planar{4, 4, 0.35, 0.20, 3};
  smc::ReachingGains vertical{12, 8, 0.8, 0.20, 8};
  smc::ReachingGains angular{10, 6, 1.0, 0.30, 60};
  smc::ReachingGains swing{15, 10, 1.5, 0.30, 120};
  double mpc_update_period = 0.01;
  mpc::MpcConfig body_mpc{30, 256.0, 32.0, 1.0, 0.25, 0.08, -0.3, 0.3, 60, 1e-5};
  mpc::MpcConfig vertical_mpc{30, 9216.0, 208.0, 1.0, 0.5, 0.5, -8.0, 8.0, 60, 1e-5};
  mpc::MpcConfig attitude_mpc{30, 6000.0, 200.0, 0.2, 0.3, 5.0, -60.0, 60.0, 60, 1e-5};
  mpc::MpcConfig swing_mpc{30, 5000.0, 200.0, 1.0, 0.5, 1.0, -5.0, 5.0, 40, 1e-5};
};

struct ControlStatus {
  double time = 0;
  double x = 0, y = 0, height = 0, roll = 0, pitch = 0, yaw = 0;
  double speed = 0, goal_distance = 0, force_residual = 0, max_torque = 0;
  int support_feet = 4;
  int contact_feet = 0;
  std::array<double, 4> foot_heights{};
  std::array<double, 3> body_acceleration{}, attitude_acceleration{};
  std::array<double, 12> swing_acceleration{};
  std::size_t waypoint = 0;
  bool complete = false, fallen = false;
};

// MuJoCo adapter: only writes motor ctrl. Uses simulator ground-truth state.
// Flat ground, slow gait; not a real-robot localization/whole-body stack.
class Go2Controller {
 public:
  Go2Controller(const mjModel* model, std::vector<navigation::Waypoint> waypoints,
                 Go2Config config = {}, ControlLaw control_law = ControlLaw::Smc);
  void reset(const mjData* data);
  // World-frame command, used by ROS /cmd_vel adapter. Rate and speed limited.
  void set_velocity(double vx, double vy, double yaw_rate);
  void update(mjData* data, Mode mode);
  [[nodiscard]] const ControlStatus& status() const { return status_; }
  [[nodiscard]] ControlLaw control_law() const { return control_law_; }
  [[nodiscard]] const auto& waypoints() const { return navigator_.waypoints(); }
 private:
  struct Leg {
    int geom = -1;
    std::array<int, 3> actuator{}, qpos{}, dof{};
    bool swing = false;
    double lift_time = 0;
    locomotion::Vec3 nominal, start, end;
  };
  const mjModel* model_;
  Go2Config config_;
  ControlLaw control_law_;
  int base_ = -1;
  int floor_ = -1;
  double mass_ = 0;
  double reset_time_ = 0;
  double yaw_reference_ = 0;
  double mpc_gait_start_time_ = -1.0;
  locomotion::Vec3 requested_velocity_ = locomotion::Vec3::Zero();
  locomotion::Vec3 filtered_velocity_ = locomotion::Vec3::Zero();
  locomotion::Vec3 reference_ = locomotion::Vec3::Zero();
  std::array<Leg, 4> legs_;
  navigation::WaypointNavigator navigator_;
  locomotion::ForceAllocator allocator_;
  smc::ReachingLaw planar_, vertical_, angular_, swing_;
  std::array<pid::PidController, 2> planar_pid_;
  std::array<pid::PidController, 1> vertical_pid_;
  std::array<pid::PidController, 3> angular_pid_;
  std::array<std::array<pid::PidController, 3>, 4> swing_pid_;
  std::array<mpc::MpcController, 3> body_mpc_;
  std::array<mpc::MpcController, 3> attitude_mpc_;
  std::array<std::array<mpc::MpcController, 3>, 4> swing_mpc_;
  double last_mpc_update_ = -1.0;
  locomotion::Vec3 mpc_body_acceleration_ = locomotion::Vec3::Zero();
  locomotion::Vec3 mpc_attitude_acceleration_ = locomotion::Vec3::Zero();
  std::array<locomotion::Vec3, 4> mpc_swing_acceleration_{};
  ControlStatus status_;
};
}  // namespace simulation
