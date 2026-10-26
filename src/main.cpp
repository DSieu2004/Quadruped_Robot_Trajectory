#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include "simulation/go2_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
struct Viewer {
  mjModel* model;
  mjData* data;
  mjvCamera camera{};
  mjvOption option{};
  mjvScene scene{};
  mjrContext context{};
  simulation::Go2Controller* controller = nullptr;
  simulation::Mode mode = simulation::Mode::Navigate;
  bool paused = true;
  bool follow = true;
  std::size_t step_count = 0;
  double mouse_x = 0;
  double mouse_y = 0;

  void reset() {
    mj_resetDataKeyframe(model, data, 0);
    // Menagerie's home key contains ctrl values; the controller starts from zero.
    mju_zero(data->ctrl, model->nu);
    mj_forward(model, data);
    if (controller) controller->reset(data);
    step_count = 0;
  }

  void step() {
    mj_step1(model, data);
    controller->update(data, mode);
    mj_step2(model, data);
    ++step_count;
  }

  static Viewer& from(GLFWwindow* window) {
    return *static_cast<Viewer*>(glfwGetWindowUserPointer(window));
  }

  static void key(GLFWwindow* window, int key, int, int action, int) {
    if (action != GLFW_PRESS) return;
    auto& viewer = from(window);
    if (key == GLFW_KEY_ESCAPE) glfwSetWindowShouldClose(window, GLFW_TRUE);
    if (key == GLFW_KEY_SPACE) viewer.paused = !viewer.paused;
    if (key == GLFW_KEY_R) viewer.reset();
    if (key == GLFW_KEY_F) viewer.follow = !viewer.follow;
    if (key == GLFW_KEY_N && viewer.paused) viewer.step();
    if (key == GLFW_KEY_0 || key == GLFW_KEY_1 || key == GLFW_KEY_2) {
      viewer.mode = key == GLFW_KEY_0 ? simulation::Mode::Passive :
                    key == GLFW_KEY_1 ? simulation::Mode::Stand : simulation::Mode::Navigate;
      std::cout << "Keyboard selected " << simulation::mode_name(viewer.mode) << "; resetting robot." << std::endl;
      viewer.reset();
    }
    const std::string law = viewer.controller
        ? simulation::control_law_name(viewer.controller->control_law()) : "SMC";
    const std::string title = viewer.paused ? "Unitree Go2 | PAUSED | Space: play | R: reset"
                                            : "Unitree Go2 | " + law + " | Space: pause | R: reset";
    glfwSetWindowTitle(window, title.c_str());
  }

  static void mouse_button(GLFWwindow* window, int, int, int) {
    auto& viewer = from(window);
    glfwGetCursorPos(window, &viewer.mouse_x, &viewer.mouse_y);
  }

  static void mouse_move(GLFWwindow* window, double x, double y) {
    auto& viewer = from(window);
    const double dx = x - viewer.mouse_x;
    const double dy = y - viewer.mouse_y;
    viewer.mouse_x = x;
    viewer.mouse_y = y;
    const bool left = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
    const bool right = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
    const bool middle = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
    if (!left && !right && !middle) return;
    int width, height;
    glfwGetWindowSize(window, &width, &height);
    if (height <= 0) return;
    const bool shift = glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                       glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;
    const int action = right ? (shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V)
                             : left ? (shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V)
                                    : mjMOUSE_ZOOM;
    mjv_moveCamera(viewer.model, action, dx / height, dy / height, &viewer.scene, &viewer.camera);
  }

  static void scroll(GLFWwindow* window, double, double offset) {
    auto& viewer = from(window);
    mjv_moveCamera(viewer.model, mjMOUSE_ZOOM, 0, -0.05 * offset, &viewer.scene, &viewer.camera);
  }
};

void verify_state(const mjModel* model, const mjData* data) {
  for (int i = 0; i < model->nq; ++i)
    if (!std::isfinite(data->qpos[i])) throw std::runtime_error("Non-finite position");
  for (int i = 0; i < model->nv; ++i)
    if (!std::isfinite(data->qvel[i])) throw std::runtime_error("Non-finite velocity");
  for (int i = 0; i < mjNWARNING; ++i)
    if (data->warning[i].number) throw std::runtime_error("MuJoCo simulation warning " + std::to_string(i));
}

int run(int argc, char** argv) {
  std::string model_path = QUADRUPED_MODEL_PATH;
  bool headless = false;
  bool check = false, autostart = false;
  int steps = 1000;
  double duration = 0;
  simulation::Mode mode = simulation::Mode::Navigate;
  simulation::ControlLaw control_law = simulation::ControlLaw::Smc;
  std::vector<navigation::Waypoint> waypoints;
  std::string log_path;
  auto parse_number = [](const char* text) {
    std::size_t consumed = 0;
    const std::string value(text);
    const double number = std::stod(value, &consumed);
    if (!std::isfinite(number) || consumed != value.size()) throw std::invalid_argument("Invalid number: " + value);
    return number;
  };
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--headless") headless = true;
    else if (arg == "--check") check = true;
    else if (arg == "--autostart") autostart = true;
    else if (arg == "--duration" && i + 1 < argc) {
      duration = parse_number(argv[++i]);
      if (duration <= 0 || duration > 3600) throw std::invalid_argument("Duration must be in (0, 3600] seconds");
    }
    else if (arg == "--log" && i + 1 < argc) log_path = argv[++i];
    else if (arg == "--waypoint" && i + 2 < argc) {
      const double x = parse_number(argv[++i]);
      const double y = parse_number(argv[++i]);
      waypoints.push_back({x, y});
    }
    else if (arg == "--controller" && i + 1 < argc) {
      const std::string value = argv[++i];
      if (value == "passive") mode = simulation::Mode::Passive;
      else if (value == "stand") mode = simulation::Mode::Stand;
      else if (value == "navigate") mode = simulation::Mode::Navigate;
      else throw std::invalid_argument("Controller must be passive, stand or navigate");
    }
    else if (arg == "--control-law" && i + 1 < argc) {
      const std::string value = argv[++i];
      if (value == "smc") control_law = simulation::ControlLaw::Smc;
      else if (value == "pid") control_law = simulation::ControlLaw::Pid;
      else if (value == "mpc") control_law = simulation::ControlLaw::Mpc;
      else throw std::invalid_argument("Control law must be smc, pid or mpc");
    }
    else if (arg == "--model" && i + 1 < argc) model_path = argv[++i];
    else if (arg == "--steps" && i + 1 < argc) {
      const std::string value = argv[++i];
      std::size_t consumed = 0;
      steps = std::stoi(value, &consumed);
      if (steps < 1 || consumed != value.size()) throw std::invalid_argument("--steps must be a positive integer");
    } else if (arg == "--help") {
      std::cout << "quadruped_viewer [--model scene.xml] [--headless] [--steps N | --duration SEC]\n"
                   "[--controller passive|stand|navigate] [--control-law smc|pid|mpc]\n"
                   "[--waypoint X Y ...] [--log FILE] [--check] [--autostart]\n"
                   "Space: pause/play | R: reset | N: step | F: follow camera | 0/1/2: passive/stand/navigate + reset | Esc: exit\n"
                   "Left drag: orbit | Right drag: pan | Scroll: zoom | Shift: alternate axis\n";
      return 0;
    } else throw std::invalid_argument("Unknown or incomplete argument: " + arg);
  }
  if (check && !headless && duration <= 0) throw std::invalid_argument("GUI --check requires --duration");
  if (waypoints.empty()) waypoints = {{0.6, 0}, {0.6, 0.4}};

  if (mj_version() != mjVERSION_HEADER) throw std::runtime_error("MuJoCo header/library version mismatch");
  char error[2048] = {};
  std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
      mj_loadXML(model_path.c_str(), nullptr, error, sizeof(error)), mj_deleteModel);
  if (!model) throw std::runtime_error("Cannot load " + model_path + ": " + error);
  std::unique_ptr<mjData, decltype(&mj_deleteData)> data(mj_makeData(model.get()), mj_deleteData);
  if (!data) throw std::runtime_error("Cannot allocate MuJoCo data");
  if (model->nq != 19 || model->nv != 18 || model->nu != 12 || model->nkey < 1)
    throw std::runtime_error("Expected a floating-base quadruped: nq=19, nv=18, nu=12 and an initial keyframe");

  Viewer viewer{model.get(), data.get()};
  simulation::Go2Controller controller(model.get(), waypoints, {}, control_law);
  viewer.controller = &controller;
  viewer.mode = mode;
  viewer.paused = !autostart;
  viewer.reset();
  if (duration > 0) steps = static_cast<int>(std::ceil(duration / model->opt.timestep));
  std::ofstream log;
  if (!log_path.empty()) {
    log.open(log_path);
    if (!log) throw std::runtime_error("Cannot open log " + log_path);
    log << "time,x,y,z,roll,pitch,yaw,speed,waypoint,distance,planned_stance_feet,wrench_residual,max_torque,complete,fallen,contact_feet,FL_foot_z,FR_foot_z,RL_foot_z,RR_foot_z";
    for (int a = 0; a < model->nu; ++a) log << ",tau_" << mj_id2name(model.get(), mjOBJ_ACTUATOR, a);
    log << ",controller_mode\n";
    log << std::setprecision(9);
  }
  auto record = [&]() {
    if (!log.is_open()) return;
    const auto& s = controller.status();
    log << s.time << ',' << s.x << ',' << s.y << ',' << s.height << ',' << s.roll << ',' << s.pitch << ','
        << s.yaw << ',' << s.speed << ',' << s.waypoint << ',' << s.goal_distance << ',' << s.support_feet << ','
        << s.force_residual << ',' << s.max_torque << ',' << s.complete << ',' << s.fallen << ',' << s.contact_feet;
    for (double z : s.foot_heights) log << ',' << z;
    for (int a = 0; a < model->nu; ++a) log << ',' << data->ctrl[a];
    log << ',' << static_cast<int>(viewer.mode) << '\n';
  };
  std::cout << "MuJoCo " << mj_versionString() << " | " << model_path << '\n'
            << "12 actuators | nq=" << model->nq << " | nv=" << model->nv << std::endl;
  if (headless) {
    for (int i = 0; i < steps; ++i) {
      viewer.step();
      verify_state(model.get(), data.get());
      if (i % 10 == 0) record();
      if (check && mode != simulation::Mode::Passive && controller.status().fallen)
        throw std::runtime_error("Robot fell at t=" + std::to_string(data->time));
    }
    const double expected_time = steps * model->opt.timestep;
    if (std::abs(data->time - expected_time) > 1e-8 * expected_time)
      throw std::runtime_error("Simulation time did not advance as expected");
    const auto& s = controller.status();
    const auto final_goal = waypoints.back();
    const double final_error = std::hypot(s.x - final_goal.x, s.y - final_goal.y);
    std::cout << simulation::mode_name(mode) << ": " << steps << " steps, t=" << data->time
              << " s | position=" << s.x << ',' << s.y << ',' << s.height
              << " | tilt=" << s.roll << ',' << s.pitch << " | speed=" << s.speed
              << " | waypoint=" << s.waypoint << '/' << waypoints.size()
              << " | final_error=" << final_error << " | complete=" << s.complete << '\n';
    if (check && mode != simulation::Mode::Passive &&
        (s.height < 0.24 || std::abs(s.roll) > 0.2 || std::abs(s.pitch) > 0.2))
      throw std::runtime_error("Final standing posture outside tolerance");
    if (check && mode == simulation::Mode::Navigate && (!s.complete || final_error > 0.10 || s.speed > 0.08))
      throw std::runtime_error("Navigation did not reach and stop at final waypoint");
    std::cout << "PASS: finite state, expected simulation time, no MuJoCo warnings.\n";
    return 0;
  }

#ifdef QUADRUPED_USE_NVIDIA
  // Configure PRIME before GLFW loads the graphics driver, for every launch path.
  if (setenv("__NV_PRIME_RENDER_OFFLOAD", "1", 1) != 0 ||
      setenv("__GLX_VENDOR_LIBRARY_NAME", "nvidia", 1) != 0)
    throw std::runtime_error("Cannot configure NVIDIA PRIME rendering");
#endif
  glfwSetErrorCallback([](int code, const char* message) {
    std::cerr << "GLFW " << code << ": " << message << '\n';
  });
  if (!glfwInit()) throw std::runtime_error("GLFW initialization failed; check the desktop DISPLAY");
  struct GlfwGuard { ~GlfwGuard() { glfwTerminate(); } } glfw_guard;
  const std::string active_law = simulation::control_law_name(control_law);
  std::unique_ptr<GLFWwindow, decltype(&glfwDestroyWindow)> window(
      glfwCreateWindow(1280, 800, autostart ? ("Unitree Go2 | " + active_law + " | Space: pause | R: reset").c_str()
                                           : "Unitree Go2 | PAUSED | Space: play | R: reset", nullptr, nullptr),
      glfwDestroyWindow);
  if (!window) throw std::runtime_error("Cannot create OpenGL window");
  glfwMakeContextCurrent(window.get());
  const auto* vendor = reinterpret_cast<const char*>(glGetString(GL_VENDOR));
  const auto* renderer = reinterpret_cast<const char*>(glGetString(GL_RENDERER));
  if (!vendor || !renderer) throw std::runtime_error("Cannot query OpenGL GPU");
  std::cout << "OpenGL vendor: " << vendor << "\nOpenGL renderer: " << renderer << std::endl;
#ifdef QUADRUPED_USE_NVIDIA
  if (std::string(vendor).find("NVIDIA") == std::string::npos)
    throw std::runtime_error("NVIDIA rendering required, but active GPU is " + std::string(renderer));
#endif
  glfwSwapInterval(1);
  mjv_defaultCamera(&viewer.camera);
  mjv_defaultOption(&viewer.option);
  viewer.option.geomgroup[3] = 0;  // Show Menagerie's visual meshes, hide collision shapes.
  mjv_defaultScene(&viewer.scene);
  mjr_defaultContext(&viewer.context);
  mjv_makeScene(model.get(), &viewer.scene, 2000);
  mjr_makeContext(model.get(), &viewer.context, mjFONTSCALE_150);
  viewer.camera.azimuth = 135;
  viewer.camera.elevation = -22;
  viewer.camera.distance = 1.8;
  viewer.camera.lookat[2] = 0.25;

  glfwSetWindowUserPointer(window.get(), &viewer);
  glfwSetKeyCallback(window.get(), Viewer::key);
  glfwSetMouseButtonCallback(window.get(), Viewer::mouse_button);
  glfwSetCursorPosCallback(window.get(), Viewer::mouse_move);
  glfwSetScrollCallback(window.get(), Viewer::scroll);
  std::cout << "Viewer ready. Space: play, R: reset, 0/1/2: passive/stand/navigate, Esc: exit.\n"
            << "Controller: " << simulation::control_law_name(control_law) << " "
            << simulation::mode_name(viewer.mode) << std::endl;
  if (viewer.mode != mode) throw std::runtime_error("Viewer controller mode changed during graphics initialization");

  auto previous = std::chrono::steady_clock::now();
  double accumulator = 0;
  while (!glfwWindowShouldClose(window.get())) {
    const auto now = std::chrono::steady_clock::now();
    const double elapsed = std::chrono::duration<double>(now - previous).count();
    previous = now;
    if (viewer.paused) accumulator = 0;
    else {
      accumulator += std::min(elapsed, 0.05);
      while (accumulator >= model->opt.timestep) {
        viewer.step();
        if (viewer.step_count % 10 == 0) record();
        accumulator -= model->opt.timestep;
      }
    }
    mjrRect viewport{0, 0, 0, 0};
    glfwGetFramebufferSize(window.get(), &viewport.width, &viewport.height);
    if (viewer.follow) {
      for (int k = 0; k < 2; ++k) viewer.camera.lookat[k] += 0.1 * (data->qpos[k] - viewer.camera.lookat[k]);
    }
    mjv_updateScene(model.get(), data.get(), &viewer.option, nullptr, &viewer.camera, mjCAT_ALL, &viewer.scene);
    // Waypoint markers are render-only geoms: no model/contact modifications.
    for (std::size_t i = 0; i < waypoints.size() && viewer.scene.ngeom < viewer.scene.maxgeom; ++i) {
      const mjtNum size[3] = {0.06, 0.06, 0.008};
      const mjtNum position[3] = {waypoints[i].x, waypoints[i].y, 0.01};
      const float color[4] = {i < controller.status().waypoint ? 0.1f : 0.9f, 0.7f, 0.2f, 0.8f};
      mjv_initGeom(&viewer.scene.geoms[viewer.scene.ngeom++], mjGEOM_CYLINDER, size, position, nullptr, color);
    }
    mjr_render(viewport, &viewer.scene, &viewer.context);
    const auto& s = controller.status();
    const std::string overlay = std::string(simulation::mode_name(viewer.mode)) + (s.fallen ? " | FALLEN - R: reset" : "") +
        (s.complete ? " | ROUTE COMPLETE" : "") + "\nWaypoint: " + std::to_string(s.waypoint) + "/" + std::to_string(waypoints.size()) +
        " | distance: " + std::to_string(s.goal_distance) + " m\n0: passive | 1: stand | 2: navigate | R: reset | F: follow camera\nGPU: " + renderer;
    mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport,
                viewer.paused ? "PAUSED - Space: play" : "RUNNING - Space: pause",
                overlay.c_str(), &viewer.context);
    glfwSwapBuffers(window.get());
    glfwPollEvents();
    if (check && viewer.mode != simulation::Mode::Passive && controller.status().fallen)
      throw std::runtime_error("Robot fell in graphical simulation at t=" + std::to_string(data->time));
    if (duration > 0 && data->time >= duration) glfwSetWindowShouldClose(window.get(), GLFW_TRUE);
  }
  mjr_freeContext(&viewer.context);
  mjv_freeScene(&viewer.scene);
  if (check) {
    const auto& s = controller.status();
    const double error = std::hypot(s.x - waypoints.back().x, s.y - waypoints.back().y);
    if (s.height < 0.24 || std::abs(s.roll) > 0.2 || std::abs(s.pitch) > 0.2 ||
        (viewer.mode == simulation::Mode::Navigate && (!s.complete || error > 0.10 || s.speed > 0.08)))
      throw std::runtime_error("Graphical simulation did not meet final control criteria");
    std::cout << "GUI PASS: final error=" << error << " m, height=" << s.height
              << " m, speed=" << s.speed << " m/s." << std::endl;
  }
  return 0;
}
}  // namespace

int main(int argc, char** argv) {
  try { return run(argc, argv); }
  catch (const std::exception& error) {
    std::cerr << "Error: " << error.what() << '\n';
    return 1;
  }
}
