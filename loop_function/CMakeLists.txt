link_directories(${CMAKE_BINARY_DIR}/controllers/)

add_library(single_robot_loop_function SHARED 
  utils/utils.h
  utils/utils.cpp
  single_robot_loop_function.h
  single_robot_loop_function.cpp
)

add_library(multi_robot_loop_function SHARED 
  utils/utils.h
  utils/utils.cpp
  multi_robot_loop_function.h
  multi_robot_loop_function.cpp
)

target_link_libraries(multi_robot_loop_function
  advanced_controller
  argos3core_simulator
  argos3plugin_simulator_dynamics2d
  argos3plugin_simulator_entities
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
  argos3plugin_simulator_media
  ${GALIB_LIBRARIES})

target_link_libraries(single_robot_loop_function
  simple_controller
  argos3core_simulator
  argos3plugin_simulator_dynamics2d
  argos3plugin_simulator_entities
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
  argos3plugin_simulator_media
  ${GALIB_LIBRARIES})
