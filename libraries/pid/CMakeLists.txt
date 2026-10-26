add_library(pid STATIC src/pid_controller.cpp)
set_target_properties(pid PROPERTIES POSITION_INDEPENDENT_CODE ON)
add_library(quadruped::pid ALIAS pid)
target_include_directories(pid PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_compile_options(pid PRIVATE -Wall -Wextra -Wpedantic)
target_compile_features(pid PUBLIC cxx_std_17)
