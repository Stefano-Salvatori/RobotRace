#ifndef ADVANCED_CONTROLLER_H
#define ADVANCED_CONTROLLER_H

/*
 * Include some necessary headers.
 */
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
/* Definition of the foot-bot proximity sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>


#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>

#include <controllers/nn/perceptron.h>

/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/**
 * A controller that can also 'sense' the distance from other robot and use it as input of the neural network
 */
class AdvancedController : public CCI_Controller
{
private:
   CCI_DifferentialSteeringActuator *wheels;

   CCI_FootBotProximitySensor *proximity;

   CCI_LEDsActuator *leds;

   CCI_PositioningSensor *positioning;

   CCI_RangeAndBearingActuator *rabActuator;

   CCI_RangeAndBearingSensor *rabSensor;

   /* Wheel speeds */
   Real leftSpeed, rightSpeed = 0.0;

   Real distanceFromOpponent = -1;

   CPerceptron perceptron;

   bool useInternalSensor = false;

public:
   static const Real MAX_VELOCITY;
   static const int GENOME_SIZE;

   /* Class constructor. */
   AdvancedController();

   /* Class destructor. */
   virtual ~AdvancedController();

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

   inline const CCI_FootBotProximitySensor::TReadings &GetProximityReadings()
   {
      return proximity->GetReadings();
   }

   inline const Real GetLeftSpeed()
   {
      return leftSpeed;
   }

   inline const Real GetRightSpeed()
   {
      return rightSpeed;
   }

   inline void SetDistanceFromOpponent(Real dist) {
      distanceFromOpponent = dist;
   }
};

#endif