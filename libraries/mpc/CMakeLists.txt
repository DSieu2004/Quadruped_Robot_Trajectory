add_library(mpc STATIC src/mpc_controller.cpp)
set_target_properties(mpc PROPERTIES POSITION_INDEPENDENT_CODE ON)
add_library(quadruped::mpc ALIAS mpc)
target_include_directories(mpc PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_compile_options(mpc PRIVATE -Wall -Wextra -Wpedantic)
target_compile_features(mpc PUBLIC cxx_std_17)
