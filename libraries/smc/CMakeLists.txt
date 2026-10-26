add_library(smc STATIC
  src/joint_smc.cpp
  src/reaching_law.cpp
)
set_target_properties(smc PROPERTIES POSITION_INDEPENDENT_CODE ON)
add_library(quadruped::smc ALIAS smc)
target_include_directories(smc PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_compile_options(smc PRIVATE -Wall -Wextra -Wpedantic)
target_compile_features(smc PUBLIC cxx_std_17)
