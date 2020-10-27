add_library(genetic_controller SHARED
  nn/neural_network.h
  nn/neural_network.cpp
  nn/perceptron.h
  nn/perceptron.cpp
  genetic_controller/genetic_controller.h
  genetic_controller/genetic_controller.cpp
)

add_library(motor_schema_controller SHARED
  ../loop_function/utils/utils.h
  ../loop_function/utils/utils.cpp
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