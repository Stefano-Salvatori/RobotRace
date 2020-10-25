link_directories(${CMAKE_BINARY_DIR}/controllers/)

add_library(single_robot_loop_function SHARED 
  utils/utils.h
  utils/utils.cpp
  single_robot_loop_function.h
  single_robot_loop_function.cpp
)

target_link_libraries(single_robot_loop_function
  genetic_controller
  argos3core_simulator
  argos3plugin_simulator_dynamics2d
  argos3plugin_simulator_entities
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
  argos3plugin_simulator_media
  ${GALIB_LIBRARIES})

  add_library(race_loop_function SHARED 
  utils/utils.h
  utils/utils.cpp
  race_loop_function.h
  race_loop_function.cpp
)

target_link_libraries(race_loop_function
  genetic_controller
  argos3core_simulator
  argos3plugin_simulator_dynamics2d
  argos3plugin_simulator_entities
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
  argos3plugin_simulator_media
  ${GALIB_LIBRARIES})
