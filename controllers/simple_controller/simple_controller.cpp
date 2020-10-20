/* Include the controller definition */
#include "simple_controller.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/logging/argos_log.h>

#define COLLISION_THRESHOLD 0.9

const Real SimpleController::MAX_VELOCITY = 20.0f;

// Formula for number of parameters:
//  (Input + 1) * Output
// Since we have 24 proximity sensors and 2 wheels actuators
//  (24 + 1) * 2 = 50
const int SimpleController::GENOME_SIZE = 50;

static CRange<Real> NN_OUTPUT_RANGE(0.0f, 1.0f);
static CRange<Real> WHEEL_ACTUATION_RANGE(-SimpleController::MAX_VELOCITY, SimpleController::MAX_VELOCITY);
static const CColor FOOTBOT_COLOR = CColor::GREEN;

SimpleController::SimpleController() {}

SimpleController::~SimpleController()
{
}

/****************************************/
/****************************************/

void SimpleController::Init(TConfigurationNode &t_node)
{
    /*
    * Get sensor/actuator handles
    */
    try
    {
        proximity = GetSensor<CCI_FootBotProximitySensor>("footbot_proximity");
        leds = GetActuator<CCI_LEDsActuator>("leds");
        wheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
    }
    catch (CARGoSException &ex)
    {
        THROW_ARGOSEXCEPTION_NESTED("Error initializing sensors/actuators", ex);
    }

    try
    {
        perceptron.Init(t_node);
        leds->SetAllColors(FOOTBOT_COLOR);
    }
    catch (CARGoSException &ex)
    {
        THROW_ARGOSEXCEPTION_NESTED("Error initializing the perceptron network", ex);
    }
}

/****************************************/
/****************************************/

void SimpleController::ControlStep()
{
    /* Get readings from proximity sensor */
    const CCI_FootBotProximitySensor::TReadings &tProxReads = proximity->GetReadings();
    Real maxProximity = tProxReads[0].Value;

    /* Fill NN inputs from sensory data and find max proximity value*/
    for (size_t i = 0; i < tProxReads.size(); ++i)
    {
        perceptron.SetInput(i, tProxReads[i].Value);
        maxProximity = Max(maxProximity, tProxReads[i].Value);
    }
    if (maxProximity > 0.9)
    {
        leds->SetAllColors(CColor::RED);
    }
    else
    {
        leds->SetAllColors(FOOTBOT_COLOR);
    }

    /* Compute NN outputs */
    perceptron.ComputeOutputs();

    /*
    * Apply NN outputs to actuation
    * The NN outputs are in the range [0,1]
    */
    NN_OUTPUT_RANGE.MapValueIntoRange(
        leftSpeed,               // value to write
        perceptron.GetOutput(0), // value to read
        WHEEL_ACTUATION_RANGE    // target range
    );
    NN_OUTPUT_RANGE.MapValueIntoRange(
        rightSpeed,              // value to write
        perceptron.GetOutput(1), // value to read
        WHEEL_ACTUATION_RANGE    // target range
    );
    wheels->SetLinearVelocity(
        leftSpeed,
        rightSpeed);
}

void SimpleController::Reset()
{
    perceptron.Reset();
}

void SimpleController::Destroy()
{
    perceptron.Destroy();
}

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.
 * The string is then usable in the configuration file to refer to this
 * controller.
 * When ARGoS reads that string in the configuration file, it knows which
 * controller class to instantiate.
 */
REGISTER_CONTROLLER(SimpleController, "simple_controller")