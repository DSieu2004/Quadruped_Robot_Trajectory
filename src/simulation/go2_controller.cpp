#include "simulation/go2_controller.hpp"

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

namespace simulation {
using locomotion::Vec3;
using Mat3 = Eigen::Matrix3d;
using RowMat3 = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>;
constexpr double pi = 3.14159265358979323846;
constexpr double foot_z = 0.022;

pid::PidGains pid_gains(double kp, double ki, double kd, double output_limit,
                       double integral_limit, double filter = 0.02) {
  pid::PidGains gains;
  gains.kp = kp;
  gains.ki = ki;
  gains.kd = kd;
  gains.derivative_filter_time_constant = filter;
  gains.anti_windup_gain = 5.0;
  gains.output_min = -output_limit;
  gains.output_max = output_limit;
  gains.integral_min = -integral_limit;
  gains.integral_max = integral_limit;
  gains.max_dt = 0.25;
  return gains;
}

std::array<std::array<pid::PidController, 3>, 4> make_swing_controllers() {
  std::array<std::array<pid::PidController, 3>, 4> controllers;
  const auto gains = pid_gains(150, 2, 25, 120, 10);
  for (auto& leg : controllers)
    for (auto& axis : leg) axis = pid::PidController(gains);
  return controllers;
}

const char* mode_name(Mode mode) {
  switch (mode) {
    case Mode::Passive: return "PASSIVE";
    case Mode::Stand: return "STAND";
    case Mode::Navigate: return "NAVIGATE";
    case Mode::Velocity: return "VELOCITY";
  }
  return "UNKNOWN";
}

const char* control_law_name(ControlLaw law) {
  switch (law) {
    case ControlLaw::Smc: return "SMC";
    case ControlLaw::Pid: return "PID";
    case ControlLaw::Mpc: return "MPC";
  }
  return "UNKNOWN";
}

Go2Controller::Go2Controller(const mjModel* model, std::vector<navigation::Waypoint> points,
                             Go2Config config, ControlLaw control_law)
    : model_(model), config_(config), control_law_(control_law),
      navigator_(std::move(points), config.navigation),
      planar_(config.planar), vertical_(config.vertical), angular_(config.angular), swing_(config.swing),
      planar_pid_{{pid::PidController(pid_gains(16, 0.8, 8, 12, 2)),
                   pid::PidController(pid_gains(16, 0.8, 8, 12, 2))}},
      vertical_pid_{{pid::PidController(pid_gains(96, 2, 20, 8, 2, 0.03))}},
      angular_pid_{{pid::PidController(pid_gains(60, 1, 16, 60, 4)),
                    pid::PidController(pid_gains(60, 1, 16, 60, 4)),
                    pid::PidController(pid_gains(60, 1, 16, 60, 4))}},
      swing_pid_(make_swing_controllers()) {
  for (double value : {config.body_height, config.gait_period, config.stance_fraction, config.swing_clearance, config.settle_time})
    if (!std::isfinite(value) || value <= 0) throw std::invalid_argument("Invalid Go2 config");
  if (config.stance_fraction <= 0.5 || config.stance_fraction >= 0.9 ||
      config.body_height < 0.25 || config.body_height > 0.35 || config.gait_period < 0.3)
    throw std::invalid_argument("Go2 config outside supported slow-gait range");
  if (!model || model->nq != 19 || model->nv != 18 || model->nu != 12)
    throw std::invalid_argument("Go2 requires nq=19, nv=18, nu=12");
  if (!std::isfinite(config.mpc_update_period) || config.mpc_update_period < model->opt.timestep)
    throw std::invalid_argument("MPC update period must be finite and at least one MuJoCo step");
  base_ = mj_name2id(model, mjOBJ_BODY, "base");
  if (base_ < 0) throw std::invalid_argument("Missing Go2 base body");
  floor_ = mj_name2id(model, mjOBJ_GEOM, "floor");
  if (floor_ < 0 || model->geom_type[floor_] != mjGEOM_PLANE)
    throw std::invalid_argument("Expected Menagerie flat floor");
  mass_ = model->body_subtreemass[base_];
  body_mpc_[0] = mpc::MpcController(config_.body_mpc);
  body_mpc_[1] = mpc::MpcController(config_.body_mpc);
  body_mpc_[2] = mpc::MpcController(config_.vertical_mpc);
  for (auto& controller : attitude_mpc_)
    controller = mpc::MpcController(config_.attitude_mpc);
  for (auto& leg : swing_mpc_)
    for (auto& controller : leg) controller = mpc::MpcController(config_.swing_mpc);
  const std::array<std::string, 4> names{{"FL", "FR", "RL", "RR"}};
  const std::array<std::string, 3> joints{{"hip", "thigh", "calf"}};
  for (int i = 0; i < 4; ++i) {
    auto& leg = legs_[i];
    leg.geom = mj_name2id(model, mjOBJ_GEOM, names[i].c_str());
    if (leg.geom < 0) throw std::invalid_argument("Missing Go2 foot " + names[i]);
    for (int j = 0; j < 3; ++j) {
      const std::string name = names[i] + "_" + joints[j];
      const int actuator = mj_name2id(model, mjOBJ_ACTUATOR, name.c_str());
      const int joint = mj_name2id(model, mjOBJ_JOINT, (name + "_joint").c_str());
      if (joint < 0 || actuator < 0 || model->actuator_trntype[actuator] != mjTRN_JOINT ||
          model->actuator_trnid[2 * actuator] != joint || model->actuator_gear[6 * actuator] != 1 ||
          !model->actuator_ctrllimited[actuator])
        throw std::invalid_argument("Expected Go2 unit-gear torque motor " + name);
      leg.actuator[j] = actuator;
      leg.qpos[j] = model->jnt_qposadr[joint];
      leg.dof[j] = model->jnt_dofadr[joint];
    }
  }
}

void Go2Controller::reset(const mjData* data) {
  navigator_.reset();
  status_ = {};
  reset_time_ = data->time;
  requested_velocity_.setZero();
  filtered_velocity_.setZero();
  for (auto& controller : planar_pid_) controller.reset();
  for (auto& controller : vertical_pid_) controller.reset();
  for (auto& controller : angular_pid_) controller.reset();
  for (auto& leg : swing_pid_)
    for (auto& controller : leg) controller.reset();
  for (auto& controller : body_mpc_) controller.reset();
  for (auto& controller : attitude_mpc_) controller.reset();
  for (auto& leg : swing_mpc_)
    for (auto& controller : leg) controller.reset();
  last_mpc_update_ = -1.0;
  mpc_gait_start_time_ = -1.0;
  mpc_body_acceleration_.setZero();
  mpc_attitude_acceleration_.setZero();
  for (auto& acceleration : mpc_swing_acceleration_) acceleration.setZero();
  reference_ = Eigen::Map<const Vec3>(data->qpos);
  const Mat3 rotation = Eigen::Map<const RowMat3>(data->xmat + 9 * base_);
  yaw_reference_ = std::atan2(rotation(1, 0), rotation(0, 0));
  status_.time = data->time;
  status_.x = reference_.x(); status_.y = reference_.y(); status_.height = reference_.z();
  status_.roll = std::atan2(rotation(2, 1), rotation(2, 2));
  status_.pitch = std::asin(std::clamp(-rotation(2, 0), -1.0, 1.0));
  status_.yaw = yaw_reference_;
  status_.goal_distance = navigator_.distance(reference_.x(), reference_.y());
  status_.complete = navigator_.complete();
  for (auto& leg : legs_) {
    leg.swing = false;
    const Vec3 foot = Eigen::Map<const Vec3>(data->geom_xpos + 3 * leg.geom);
    leg.nominal = rotation.transpose() * (foot - reference_);
    leg.start = leg.end = foot;
  }
}

void Go2Controller::set_velocity(double vx, double vy, double yaw_rate) {
  if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(yaw_rate))
    throw std::invalid_argument("Non-finite velocity command");
  requested_velocity_ = Vec3(vx, vy, std::clamp(yaw_rate, -0.25, 0.25));
  const double speed = requested_velocity_.head<2>().norm();
  if (speed > config_.navigation.max_speed)
    requested_velocity_.head<2>() *= config_.navigation.max_speed / speed;
}

void Go2Controller::update(mjData* data, Mode mode) {
  const double dt = model_->opt.timestep;
  // Full-body MPC uses a crawl schedule; the validated SMC and trajectory-PID
  // configurations retain the tuned diagonal gait.
  const bool crawl_gait = control_law_ == ControlLaw::Mpc;
  const double period = crawl_gait
      ? std::max(config_.gait_period, 2.0) : config_.gait_period;
  const double duty = crawl_gait
      ? std::max(config_.stance_fraction, 0.85) : config_.stance_fraction;
  const double swing_time = period * (1 - duty);
  const double time = data->time - reset_time_;
  status_.time = data->time;
  const Vec3 position = Eigen::Map<const Vec3>(data->qpos);
  const Vec3 velocity = Eigen::Map<const Vec3>(data->qvel);
  const Mat3 rotation = Eigen::Map<const RowMat3>(data->xmat + 9 * base_);
  const Vec3 omega = rotation * Eigen::Map<const Vec3>(data->qvel + 3);
  const Vec3 angles(std::atan2(rotation(2, 1), rotation(2, 2)),
                    std::asin(std::clamp(-rotation(2, 0), -1.0, 1.0)),
                    std::atan2(rotation(1, 0), rotation(0, 0)));
  status_.x = position.x(); status_.y = position.y(); status_.height = position.z();
  status_.roll = angles.x(); status_.pitch = angles.y(); status_.yaw = angles.z();
  status_.speed = velocity.head<2>().norm();
  std::array<bool, 4> contact{};
  for (int c = 0; c < data->ncon; ++c) {
    const auto& pair = data->contact[c];
    if (pair.dist > 0.002 || pair.efc_address < 0) continue;
    for (int i = 0; i < 4; ++i)
      if ((pair.geom[0] == legs_[i].geom && pair.geom[1] == floor_) ||
          (pair.geom[1] == legs_[i].geom && pair.geom[0] == floor_)) contact[i] = true;
  }
  status_.contact_feet = static_cast<int>(std::count(contact.begin(), contact.end(), true));
  for (int i = 0; i < 4; ++i) status_.foot_heights[i] = data->geom_xpos[3 * legs_[i].geom + 2];
  status_.fallen = status_.fallen || (time > 0.3 &&
      (position.z() < 0.15 || std::abs(angles.x()) > 0.85 || std::abs(angles.y()) > 0.85));
  mju_zero(data->ctrl, model_->nu);
  if (mode == Mode::Passive || status_.fallen) return;

  navigation::Command command;
  double yaw_rate = 0;
  if (mode == Mode::Navigate && time > config_.settle_time)
    command = navigator_.update(position.x(), position.y(), status_.speed, dt);
  if (mode == Mode::Velocity && time > config_.settle_time) {
    const Vec3 delta = requested_velocity_ - filtered_velocity_;
    const double delta_xy = delta.head<2>().norm();
    if (delta_xy > 0)
      filtered_velocity_.head<2>() += delta.head<2>() * std::min(1.0, config_.navigation.max_acceleration * dt / delta_xy);
    filtered_velocity_.z() += std::clamp(delta.z(), -0.5 * dt, 0.5 * dt);
    command = {filtered_velocity_.x(), filtered_velocity_.y()};
    yaw_rate = filtered_velocity_.z();
    yaw_reference_ = std::remainder(yaw_reference_ + yaw_rate * dt, 2 * pi);
  }
  status_.waypoint = navigator_.index();
  status_.complete = navigator_.complete();
  status_.goal_distance = navigator_.distance(position.x(), position.y());
  const Vec3 desired_velocity(command.vx, command.vy, 0);
  const bool mpc_update_due = control_law_ == ControlLaw::Mpc &&
      (last_mpc_update_ < 0.0 || data->time - last_mpc_update_ >= config_.mpc_update_period - 0.5 * dt);
  const double mpc_dt = last_mpc_update_ < 0.0 ? config_.mpc_update_period
      : std::max(dt, data->time - last_mpc_update_);
  const double start_speed = mode == Mode::Velocity ? 0.005 : 0.015;
  const bool walking = desired_velocity.head<2>().norm() > start_speed || std::abs(yaw_rate) > 0.025;
  if (control_law_ == ControlLaw::Mpc && walking && mpc_gait_start_time_ < 0.0)
    mpc_gait_start_time_ = time;
  reference_.head<2>() += dt * desired_velocity.head<2>();
  // Keep the integrated reference reachable if tracking lags.
  for (int k = 0; k < 2; ++k) reference_[k] = std::clamp(reference_[k], position[k] - 0.12, position[k] + 0.12);
  reference_.z() = 0.27 + (config_.body_height - 0.27) * std::clamp(time / config_.settle_time, 0.0, 1.0);

  std::array<Vec3, 4> feet, arms;
  std::array<bool, 4> stance;
  bool crawl_swing_busy = crawl_gait &&
      std::any_of(legs_.begin(), legs_.end(), [](const Leg& leg) { return leg.swing; });
  const Vec3 com = Eigen::Map<const Vec3>(data->subtree_com + 3 * base_);
  const Mat3 yaw_rotation = Eigen::AngleAxisd(angles.z(), Vec3::UnitZ()).toRotationMatrix();
  for (int i = 0; i < 4; ++i) {
    auto& leg = legs_[i];
    feet[i] = Eigen::Map<const Vec3>(data->geom_xpos + 3 * leg.geom);
    arms[i] = feet[i] - com;
    const double offset = crawl_gait
        ? 0.25 * static_cast<double>(i)
        : ((i == 0 || i == 3) ? 0.0 : 0.5);
    const double gait_elapsed = control_law_ == ControlLaw::Mpc
        ? (mpc_gait_start_time_ < 0.0 ? 0.0 : std::max(0.0, time - mpc_gait_start_time_))
        : std::max(0.0, time - config_.settle_time);
    const double phase = std::fmod(gait_elapsed / period + offset, 1.0);
    const bool scheduled_swing = phase >= duty;
    if (leg.swing && data->time - leg.lift_time >= swing_time) {
      const bool landed = !crawl_gait ||
          (contact[i] && feet[i].z() <= foot_z + 0.025);
      if (landed) {
        leg.swing = false;
        if (crawl_gait)
          crawl_swing_busy = std::any_of(legs_.begin(), legs_.end(), [](const Leg& other) { return other.swing; });
      }
    }
    // Start only at the boundary, avoiding shortened swings when navigation starts.
    if (!leg.swing && walking && scheduled_swing && phase < duty + dt / period * 1.5 &&
        (!crawl_gait || !crawl_swing_busy)) {
      leg.swing = true;
      if (crawl_gait) crawl_swing_busy = true;
      leg.lift_time = data->time;
      for (auto& controller : swing_pid_[i]) controller.reset();
      for (auto& controller : swing_mpc_[i]) controller.reset();
      leg.start = feet[i];
      // MPC retains its separately tuned, shorter stride prediction.
      const double horizon = control_law_ == ControlLaw::Mpc
          ? config_.gait_period * (0.5 * config_.stance_fraction + 1.0 - config_.stance_fraction)
          : 0.5 * period * duty + swing_time;
      const Mat3 landing_rotation = Eigen::AngleAxisd(yaw_rate * horizon, Vec3::UnitZ()).toRotationMatrix() * yaw_rotation;
      leg.end = position + landing_rotation * leg.nominal + desired_velocity * horizon;
      leg.end.head<2>() += 0.05 * (velocity - desired_velocity).head<2>();
      leg.end.z() = foot_z;
    }
    // A scheduled stance foot remains in the allocator until the gait lifts it.
    stance[i] = !leg.swing || (control_law_ == ControlLaw::Mpc && contact[i]);
  }

  Vec3 acceleration;
  if (control_law_ == ControlLaw::Mpc && mpc_update_due) {
    for (int axis = 0; axis < 3; ++axis) {
      std::vector<mpc::Reference> references;
      references.reserve(config_.body_mpc.horizon_steps);
      for (std::size_t step = 1; step <= config_.body_mpc.horizon_steps; ++step) {
        const double future_time = static_cast<double>(step) * mpc_dt;
        const double target_position = axis < 2
            ? reference_[axis] + future_time * desired_velocity[axis]
            : reference_.z();
        const double target_velocity = axis < 2 ? desired_velocity[axis] : 0.0;
        references.push_back({target_position, target_velocity});
      }
      mpc_body_acceleration_[axis] = body_mpc_[axis].update(
          position[axis], velocity[axis], references, mpc_dt);
    }
  }
  if (control_law_ == ControlLaw::Mpc) {
    acceleration = mpc_body_acceleration_;
  } else {
    for (int k = 0; k < 2; ++k) {
      acceleration[k] = control_law_ == ControlLaw::Pid
          ? planar_pid_[k].update(reference_[k], position[k], dt)
          : planar_.acceleration(position[k] - reference_[k], velocity[k] - desired_velocity[k]);
    }
    acceleration.z() = control_law_ == ControlLaw::Pid
        ? vertical_pid_[0].update(reference_.z(), position.z(), dt)
        : vertical_.acceleration(position.z() - reference_.z(), velocity.z());
  }
  locomotion::Wrench wrench;
  // Keep attitude error, angular velocity and torque in the same WORLD frame.
  // Euler roll/pitch are body-axis errors: pairing them with world omega fails
  // after a large heading change (especially while walking at yaw = 90 deg).
  const Mat3 desired_rotation = Eigen::AngleAxisd(yaw_reference_, Vec3::UnitZ()).toRotationMatrix();
  const Eigen::AngleAxisd attitude_error(rotation * desired_rotation.transpose());
  const Vec3 rotation_error = attitude_error.angle() * attitude_error.axis();
  Vec3 angular_acceleration;
  if (control_law_ == ControlLaw::Mpc && mpc_update_due) {
    for (int axis = 0; axis < 3; ++axis) {
      std::vector<mpc::Reference> references(config_.attitude_mpc.horizon_steps, {0.0, 0.0});
      mpc_attitude_acceleration_[axis] = attitude_mpc_[axis].update(
          rotation_error[axis], omega[axis] - (axis == 2 ? yaw_rate : 0.0),
          references, mpc_dt);
    }
  }
  if (control_law_ == ControlLaw::Mpc) {
    angular_acceleration = mpc_attitude_acceleration_;
  } else {
    for (int k = 0; k < 3; ++k) {
      angular_acceleration[k] = control_law_ == ControlLaw::Pid
          ? angular_pid_[k].update(0.0, rotation_error[k], dt)
          : angular_.acceleration(rotation_error[k], omega[k] - (k == 2 ? yaw_rate : 0));
    }
  }
  // Approximate centroidal inertia (kg.m^2); the model is not canceled exactly.
  const Mat3 inertia_world = rotation * Vec3(0.25, 0.8, 0.9).asDiagonal() * rotation.transpose();
  const Vec3 gyro = omega.cross(inertia_world * omega);
  wrench.head<3>() = mass_ * (acceleration - Eigen::Map<const Vec3>(model_->opt.gravity));
  wrench.tail<3>() = inertia_world * angular_acceleration + gyro;
  const auto solution = allocator_.solve(arms, stance, wrench);
  for (int axis = 0; axis < 3; ++axis) {
    status_.body_acceleration[axis] = acceleration[axis];
    status_.attitude_acceleration[axis] = angular_acceleration[axis];
  }
  status_.force_residual = solution.residual;
  status_.support_feet = 0;
  status_.max_torque = 0;
  for (int i = 0; i < 4; ++i) {
    auto& leg = legs_[i];
    Eigen::Matrix<double, 3, 18, Eigen::RowMajor> jacobian;
    mj_jacGeom(model_, data, jacobian.data(), nullptr, leg.geom);
    Vec3 foot_force;
    Vec3 mpc_swing_force = Vec3::Zero();
    Vec3 foot_velocity = Vec3::Zero();
    if (control_law_ == ControlLaw::Mpc && leg.swing) {
      foot_velocity = jacobian * Eigen::Map<const Eigen::Matrix<double, 18, 1>>(data->qvel);
      if (mpc_update_due) {
        std::array<std::vector<mpc::Reference>, 3> references;
        for (auto& axis_refs : references) axis_refs.reserve(config_.swing_mpc.horizon_steps);
        for (std::size_t step = 1; step <= config_.swing_mpc.horizon_steps; ++step) {
          const double future_time = data->time + static_cast<double>(step) * mpc_dt;
          const double future_u = std::clamp((future_time - leg.lift_time) / swing_time, 0.0, 1.0);
          const double future_blend = future_u * future_u * future_u *
              (10.0 + future_u * (-15.0 + 6.0 * future_u));
          const double future_blend_rate = 30.0 * future_u * future_u *
              (1.0 - future_u) * (1.0 - future_u) / swing_time;
          Vec3 future_target = leg.start + future_blend * (leg.end - leg.start);
          Vec3 future_target_velocity = future_blend_rate * (leg.end - leg.start);
          future_target.z() += config_.swing_clearance * std::pow(std::sin(pi * future_u), 2);
          future_target_velocity.z() += config_.swing_clearance * pi *
              std::sin(2.0 * pi * future_u) / swing_time;
          for (int axis = 0; axis < 3; ++axis)
            references[axis].push_back({future_target[axis], future_target_velocity[axis]});
        }
        for (int axis = 0; axis < 3; ++axis) {
          mpc_swing_acceleration_[i][axis] = swing_mpc_[i][axis].update(
              feet[i][axis], foot_velocity[axis], references[axis], mpc_dt);
        }
      }
      mpc_swing_force = mpc_swing_acceleration_[i];
    }
    if (stance[i]) {
      ++status_.support_feet;
      // f is ground-on-robot. Motor torque must exert the opposite foot force.
      foot_force = -solution.forces[i];
      if (control_law_ == ControlLaw::Mpc && leg.swing && contact[i]) {
        foot_force += mpc_swing_force;
        foot_force.z() += solution.forces[i].z() + 15.0;
      }
    } else {
      const double u = std::clamp((data->time - leg.lift_time) / swing_time, 0.0, 1.0);
      const double blend = u * u * u * (10 + u * (-15 + 6 * u));
      const double blend_dot = 30 * u * u * (1 - u) * (1 - u) / swing_time;
      Vec3 target = leg.start + blend * (leg.end - leg.start);
      Vec3 target_velocity = blend_dot * (leg.end - leg.start);
      target.z() += config_.swing_clearance * std::pow(std::sin(pi * u), 2);
      target_velocity.z() += config_.swing_clearance * pi * std::sin(2 * pi * u) / swing_time;
      if (control_law_ != ControlLaw::Mpc)
        foot_velocity = jacobian * Eigen::Map<const Eigen::Matrix<double, 18, 1>>(data->qvel);
      for (int k = 0; k < 3; ++k) {
        if (control_law_ == ControlLaw::Mpc)
          foot_force[k] = mpc_swing_force[k];
        else
          foot_force[k] = control_law_ == ControlLaw::Pid
              ? swing_pid_[i][k].update(target[k], feet[i][k], dt)
              : swing_.acceleration(feet[i][k] - target[k], foot_velocity[k] - target_velocity[k]);
      }
      if (control_law_ == ControlLaw::Mpc)
        for (int axis = 0; axis < 3; ++axis)
          status_.swing_acceleration[3 * i + axis] = foot_force[axis];
    }
    for (int j = 0; j < 3; ++j) {
      const int a = leg.actuator[j], v = leg.dof[j];
      const double torque = jacobian.col(v).dot(foot_force) + data->qfrc_bias[v]
                            - data->qfrc_passive[v];
      data->ctrl[a] = std::clamp(torque, model_->actuator_ctrlrange[2 * a], model_->actuator_ctrlrange[2 * a + 1]);
      status_.max_torque = std::max(status_.max_torque, std::abs(data->ctrl[a]));
    }
  }
  if (mpc_update_due) last_mpc_update_ = data->time;
}
}  // namespace simulation
