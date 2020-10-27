#ifndef GENETIC_CONTROLLER_H
#define GENETIC_CONTROLLER_H

/*
 * Include some necessary headers.
 */
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
/* Definition of the foot-bot proximity sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_distance_scanner_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_distance_scanner_actuator.h>

#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>
#include <vector>

#include <controllers/common_controller.h>

#include <controllers/nn/perceptron.h>

/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

class GeneticController : public CommonController
{
private:
   

   /* The perceptron neural network */
   CPerceptron perceptron;

public:
   static const int GENOME_SIZE;

   /* Class constructor. */
   GeneticController();

   /* Class destructor. */
   virtual ~GeneticController();

   /*
    * This function initializes the controller.
    * The 't_node' variable points to the <parameters> section in the XML
    */
   virtual void Init(TConfigurationNode &t_node);

   /*
    * This function is called once every time step.
    * The length of the time step is set in the XML file.
    */
   virtual void ControlStep();

   /*
    * This function resets the controller to its state right after the
    * Init().
    * It is called when you press the reset button in the GUI.
    */
   virtual void Reset();

   /*
    * Called to cleanup what done by Init() when the experiment finishes.
    */
   virtual void Destroy();

   inline CPerceptron &GetPerceptron()
   {
      return perceptron;
   }
};

#endif