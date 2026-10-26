cmake_minimum_required(VERSION 3.22)
project(quadruped_smc VERSION 0.1.0 LANGUAGES CXX)
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(MUJOCO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/third_party/mujoco-3.3.5" CACHE PATH "MuJoCo SDK directory")
find_path(MUJOCO_INCLUDE_DIR mujoco/mujoco.h HINTS "${MUJOCO_ROOT}/include" REQUIRED)
find_library(MUJOCO_LIBRARY NAMES mujoco HINTS "${MUJOCO_ROOT}/lib" REQUIRED)
add_library(mujoco_sdk UNKNOWN IMPORTED GLOBAL)
set_target_properties(mujoco_sdk PROPERTIES IMPORTED_LOCATION "${MUJOCO_LIBRARY}" INTERFACE_INCLUDE_DIRECTORIES "${MUJOCO_INCLUDE_DIR}")
find_package(glfw3 3.3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
option(QUADRUPED_USE_NVIDIA "Require NVIDIA PRIME rendering for the Linux viewer" ON)

add_subdirectory(libraries/smc)
add_subdirectory(libraries/pid)
add_subdirectory(libraries/mpc)

add_library(navigation STATIC src/navigation/waypoint_navigator.cpp)
add_library(quadruped::navigation ALIAS navigation)
target_include_directories(navigation PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_compile_options(navigation PRIVATE -Wall -Wextra -Wpedantic)
target_compile_features(navigation PUBLIC cxx_std_17)

add_library(locomotion STATIC src/locomotion/force_allocator.cpp)
add_library(quadruped::locomotion ALIAS locomotion)
target_include_directories(locomotion PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(locomotion PUBLIC Eigen3::Eigen)
target_compile_options(locomotion PRIVATE -Wall -Wextra -Wpedantic)
target_compile_features(locomotion PUBLIC cxx_std_17)

add_library(go2_control STATIC src/simulation/go2_controller.cpp)
add_library(quadruped::go2_control ALIAS go2_control)
target_include_directories(go2_control PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(go2_control PUBLIC quadruped::smc quadruped::pid quadruped::mpc quadruped::navigation quadruped::locomotion mujoco_sdk)
target_compile_options(go2_control PRIVATE -Wall -Wextra -Wpedantic)

add_executable(quadruped_viewer src/main.cpp)
target_link_libraries(quadruped_viewer PRIVATE quadruped::go2_control glfw OpenGL::GL)
if(QUADRUPED_USE_NVIDIA AND CMAKE_SYSTEM_NAME STREQUAL "Linux")
  target_compile_definitions(quadruped_viewer PRIVATE QUADRUPED_USE_NVIDIA=1)
endif()
target_compile_options(quadruped_viewer PRIVATE -Wall -Wextra -Wpedantic)
target_compile_definitions(quadruped_viewer PRIVATE QUADRUPED_MODEL_PATH="${CMAKE_CURRENT_SOURCE_DIR}/models/quadruped/scene.xml")

add_executable(smc_example examples/smc_channel.cpp)
target_link_libraries(smc_example PRIVATE quadruped::smc)
add_executable(pid_example examples/pid_channel.cpp)
target_link_libraries(pid_example PRIVATE quadruped::pid)

# Reusable, simulator-independent libraries. The Go2 adapter stays in this workspace.
install(TARGETS smc pid mpc navigation locomotion EXPORT QuadrupedControlTargets
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY libraries/smc/include/smc libraries/pid/include/pid libraries/mpc/include/mpc include/navigation include/locomotion DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
configure_package_config_file(cmake/QuadrupedControlConfig.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/QuadrupedControlConfig.cmake"
  INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/QuadrupedControl")
write_basic_package_version_file("${CMAKE_CURRENT_BINARY_DIR}/QuadrupedControlConfigVersion.cmake"
  VERSION ${PROJECT_VERSION} COMPATIBILITY SameMajorVersion)
install(EXPORT QuadrupedControlTargets NAMESPACE quadruped:: DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/QuadrupedControl")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/QuadrupedControlConfig.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/QuadrupedControlConfigVersion.cmake"
  DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/QuadrupedControl")

include(CTest)
if(BUILD_TESTING)
  add_executable(smc_test tests/smc_test.cpp)
  target_link_libraries(smc_test PRIVATE quadruped::smc)
  add_test(NAME smc_control_law COMMAND smc_test)
  add_test(NAME model_simulation COMMAND quadruped_viewer --headless --controller passive --steps 2000)
  add_executable(control_test tests/control_test.cpp)
  target_link_libraries(control_test PRIVATE quadruped::smc quadruped::navigation quadruped::locomotion)
  add_test(NAME control_libraries COMMAND control_test)
  add_executable(go2_integration_test tests/go2_integration_test.cpp)
  target_link_libraries(go2_integration_test PRIVATE quadruped::go2_control)
  target_compile_definitions(go2_integration_test PRIVATE QUADRUPED_MODEL_PATH="${CMAKE_CURRENT_SOURCE_DIR}/models/quadruped/scene.xml")
  add_test(NAME go2_physical_control COMMAND go2_integration_test)
  set_tests_properties(go2_physical_control PROPERTIES TIMEOUT 60)
  add_executable(go2_heading_test tests/go2_heading_test.cpp)
  target_link_libraries(go2_heading_test PRIVATE quadruped::go2_control)
  target_compile_definitions(go2_heading_test PRIVATE QUADRUPED_MODEL_PATH="${CMAKE_CURRENT_SOURCE_DIR}/models/quadruped/scene.xml")
  add_test(NAME go2_heading_control COMMAND go2_heading_test)
  set_tests_properties(go2_heading_control PROPERTIES TIMEOUT 60)
endif()
