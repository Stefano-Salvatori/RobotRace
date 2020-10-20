/* Include the controller definition */
#include "advanced_controller.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/logging/argos_log.h>

#define COLLISION_THRESHOLD 0.9

const Real AdvancedController::MAX_VELOCITY = 20.0f;

// Formula for number of parameters:
//  (Input + 1) * Output
// Since we have 24 proximity sensors, 1 input for the distance from the other robot and 2 wheels actuators
//  (25 + 1) * 2 = 52
const int AdvancedController::GENOME_SIZE = 52;

static CRange<Real> NN_OUTPUT_RANGE(0.0f, 1.0f);
static CRange<Real> WHEEL_ACTUATION_RANGE(-AdvancedController::MAX_VELOCITY, AdvancedController::MAX_VELOCITY);
static const CColor FOOTBOT_COLOR = CColor::GREEN;

AdvancedController::AdvancedController() {}

AdvancedController::~AdvancedController()
{
}

/****************************************/
/****************************************/

void AdvancedController::Init(TConfigurationNode &t_node)
{
    /*
    * Get sensor/actuator handles
    */
    try
    {
        proximity = GetSensor<CCI_FootBotProximitySensor>("footbot_proximity");
        leds = GetActuator<CCI_LEDsActuator>("leds");
        wheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
        positioning = GetSensor<CCI_PositioningSensor>("positioning");
        rabActuator = GetActuator<CCI_RangeAndBearingActuator>("range_and_bearing");
        rabSensor = GetSensor<CCI_RangeAndBearingSensor>("range_and_bearing");
        GetNodeAttributeOrDefault(t_node, "useRAB", useInternalSensor, false);
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

void AdvancedController::ControlStep()
{

    /* Send our current position to the other robots*/
    rabActuator->SetData(0, 1);

    /* Get readings from proximity sensor */
    const CCI_FootBotProximitySensor::TReadings &tProxReads = proximity->GetReadings();

    /* Fill NN inputs from sensory data and find max proximity value*/
    for (size_t i = 0; i < tProxReads.size(); ++i)
    {
        perceptron.SetInput(i, tProxReads[i].Value);
    }

    /* Check if there is another robot nearby */
    const CCI_RangeAndBearingSensor::TReadings &rabReads = rabSensor->GetReadings();
    Real distanceSensed = -1;
    for (size_t i = 0; i < rabReads.size(); i++)
    {
        distanceSensed = rabReads[i].Range;
    }

    if (useInternalSensor)
    {
        LOG << distanceSensed << std::endl;
        perceptron.SetInput(perceptron.GetNumberOfInputs() - 1, distanceSensed / 300);
    }
    else
    {
        perceptron.SetInput(perceptron.GetNumberOfInputs() - 1, distanceFromOpponent);
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

void AdvancedController::Reset()
{
    perceptron.Reset();
}

void AdvancedController::Destroy()
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
REGISTER_CONTROLLER(AdvancedController, "advanced_controller")