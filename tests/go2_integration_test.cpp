#include "simulation/go2_controller.hpp"

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>

void check(bool condition, const char* message) { if (!condition) throw std::runtime_error(message); }

int main() {
  char error[2048] = {};
  std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(mj_loadXML(QUADRUPED_MODEL_PATH, nullptr, error, sizeof(error)), mj_deleteModel);
  if (!model) throw std::runtime_error(error);
  std::unique_ptr<mjData, decltype(&mj_deleteData)> data(mj_makeData(model.get()), mj_deleteData);
  const int base = mj_name2id(model.get(), mjOBJ_BODY, "base");
  simulation::Go2Controller controller(model.get(), {{0.45, 0}, {0.45, 0.3}});
  auto reset = [&]() {
    mj_resetDataKeyframe(model.get(), data.get(), 0);
    mju_zero(data->ctrl, model->nu);
    mj_forward(model.get(), data.get());
    controller.reset(data.get());
  };
  for (int scenario = 0; scenario < 2; ++scenario) {
    reset();
    const bool disturbed = scenario == 0;
    if (disturbed) {
      const Eigen::Quaterniond q(Eigen::AngleAxisd(0.05, Eigen::Vector3d::UnitX()) *
                                 Eigen::AngleAxisd(-0.035, Eigen::Vector3d::UnitY()));
      data->qpos[3] = q.w(); data->qpos[4] = q.x(); data->qpos[5] = q.y(); data->qpos[6] = q.z();
      data->qvel[0] = 0.04; data->qvel[1] = -0.02;
      mj_forward(model.get(), data.get());
      controller.reset(data.get());
    }
    const int steps = static_cast<int>((disturbed ? 12 : 25) / model->opt.timestep);
    double max_tilt = 0, max_foot_height = 0;
    int lifted_samples = 0;
    for (int i = 0; i < steps; ++i) {
      mj_step1(model.get(), data.get());
      // This is an explicit test disturbance, never a supporting/propulsive force.
      data->xfrc_applied[6 * base] = disturbed && data->time > 5 && data->time < 5.15 ? 12 : 0;
      data->xfrc_applied[6 * base + 1] = disturbed && data->time > 5 && data->time < 5.15 ? 8 : 0;
      controller.update(data.get(), disturbed ? simulation::Mode::Stand : simulation::Mode::Navigate);
      const auto& status = controller.status();
      check(!status.fallen, "Robot fell during integration test");
      for (int a = 0; a < model->nu; ++a)
        check(std::isfinite(data->ctrl[a]) && data->ctrl[a] >= model->actuator_ctrlrange[2 * a] &&
              data->ctrl[a] <= model->actuator_ctrlrange[2 * a + 1], "Motor torque outside model limits");
      for (int v = 0; v < model->nv; ++v) check(data->qfrc_applied[v] == 0, "Unexpected applied generalized force");
      if (!disturbed)
        for (int b = 0; b < 6 * model->nbody; ++b) check(data->xfrc_applied[b] == 0, "Unexpected applied body force");
      max_tilt = std::max({max_tilt, std::abs(status.roll), std::abs(status.pitch)});
      max_foot_height = std::max(max_foot_height, *std::max_element(status.foot_heights.begin(), status.foot_heights.end()));
      if (status.contact_feet < 4 && status.support_feet < 4) ++lifted_samples;
      mj_step2(model.get(), data.get());
      for (int q = 0; q < model->nq; ++q) check(std::isfinite(data->qpos[q]), "Non-finite robot state");
      for (int w = 0; w < mjNWARNING; ++w) check(data->warning[w].number == 0, "MuJoCo warning");
    }
    const auto& s = controller.status();
    check(s.height > 0.28 && s.height < 0.34 && std::abs(s.roll) < 0.03 && std::abs(s.pitch) < 0.03,
          "Final body posture outside tolerance");
    check(max_tilt < 0.35 && s.speed < 0.03, "Posture or settling failure");
    if (disturbed) check(std::hypot(s.x, s.y) < 0.04, "Failed to recover from standing disturbance");
    else {
      check(s.complete && s.waypoint == 2 && std::hypot(s.x - 0.45, s.y - 0.3) < 0.09, "Route not completed");
      check(lifted_samples > 100 && max_foot_height > 0.055, "No physical foot lifting observed");
    }
    std::cout << (disturbed ? "DISTURBED STAND" : "TWO-WAYPOINT ROUTE")
              << " position=" << s.x << ',' << s.y << " height=" << s.height
              << " max_tilt_rad=" << max_tilt << " speed=" << s.speed
              << " max_foot_z=" << max_foot_height << " lifted_samples=" << lifted_samples << '\n';
  }
  reset();
  controller.update(data.get(), simulation::Mode::Stand);
  check(!controller.status().complete && controller.status().waypoint == 0 && !controller.status().fallen,
        "Controller reset did not clear route/fall state");
  std::cout << "PASS: physical locomotion, disturbance recovery, torque limits and reset.\n";
}
