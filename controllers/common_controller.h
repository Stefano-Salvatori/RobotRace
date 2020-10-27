#ifndef COMMON_CONTROLLER_H
#define COMMON_CONTROLLER_H

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




/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

class CommonController : public CCI_Controller
{
protected:
   /* Pointer to the differential steering actuator */
   CCI_DifferentialSteeringActuator *wheels;

   /* Pointer to the foot-bot proximity sensor */
   CCI_FootBotProximitySensor *proximity;

   CCI_FootBotDistanceScannerSensor *distanceScannerSensor;
   CCI_FootBotDistanceScannerActuator *distanceScannerActuator;

   /* Pointer to the foot-bot proximity sensor */
   CCI_LEDsActuator *leds;

   /*
    * The following variables are used as parameters for the
    * algorithm. You can set their value in the <parameters> section
    * of the XML configuration file, under the
    * <controllers><footbot_diffusion_controller> section.
    */

   /* Wheel speeds */
   Real leftSpeed, rightSpeed = 0.0;

public:
   static const Real MAX_VELOCITY;
   /* Class constructor. */
   CommonController();

   /* Class destructor. */
   virtual ~CommonController();

   /*
    * This function initializes the controller.
    * The 't_node' variable points to the <parameters> section in the XML
    */
   virtual void Init(TConfigurationNode &t_node) = 0;

   /*
    * This function is called once every time step.
    * The length of the time step is set in the XML file.
    */
   virtual void ControlStep() = 0;

   /*
    * This function resets the controller to its state right after the
    * Init().
    * It is called when you press the reset button in the GUI.
    */
   virtual void Reset() = 0;

   /*
    * Called to cleanup what done by Init() when the experiment finishes.
    */
   virtual void Destroy() = 0;

   /**
    * Get the max proximity value read from the sensor
    */
   inline const Real GetMaxProximityValue()
   {
      const CCI_FootBotProximitySensor::TReadings &proxReads = proximity->GetReadings();
      Real maxProximity = proxReads[0].Value;
      for (size_t i = 0; i < proxReads.size(); ++i)
      {
         maxProximity = Max(maxProximity, proxReads[i].Value);
      }
      return maxProximity;
   }

   inline const CCI_FootBotProximitySensor::TReadings &GetProximityReadings()
   {
      return proximity->GetReadings();
   }

   inline const Real GetLeftSpeed()
   {
      return this->leftSpeed;
   }

   inline const Real GetRightSpeed()
   {
      return this->rightSpeed;
   }
};

#endif