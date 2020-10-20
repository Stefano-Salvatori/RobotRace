add_library(simple_controller SHARED
  nn/neural_network.h
  nn/neural_network.cpp
  nn/perceptron.h
  nn/perceptron.cpp
  simple_controller/simple_controller.h
  simple_controller/simple_controller.cpp
)

add_library(advanced_controller SHARED
  nn/neural_network.h
  nn/neural_network.cpp
  nn/perceptron.h
  nn/perceptron.cpp
  advanced_controller/advanced_controller.h
  advanced_controller/advanced_controller.cpp
)

target_link_libraries(advanced_controller
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot)

target_link_libraries(simple_controller
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot)