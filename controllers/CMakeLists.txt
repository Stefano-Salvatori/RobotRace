add_library(genetic_controller SHARED
  common_controller.h
  common_controller.cpp
  nn/neural_network.h
  nn/neural_network.cpp
  nn/perceptron.h
  nn/perceptron.cpp
  genetic_controller/genetic_controller.h
  genetic_controller/genetic_controller.cpp
)

add_library(common_controller SHARED
  common_controller.h
  common_controller.cpp
)

add_library(motor_schema_controller SHARED
  common_controller.h
  common_controller.cpp
  motor_schema_controller/motor_schema.h
  motor_schema_controller/motor_schema.cpp
)

target_link_libraries(genetic_controller
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot)

  target_link_libraries(motor_schema_controller
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot)