/* Include the controller definition */
#include "advanced_controller.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/logging/argos_log.h>

#define COLLISION_THRESHOLD 0.9
#define SHORT_RANGE_MAX_DISTANCE 30
#define LONG_RANGE_MAX_DISTANCE 150
#define ROTATION_SPEED 50

const Real AdvancedController::MAX_VELOCITY = 20.0f;

// Formula for number of parameters:
//  (Input + 1) * Output
// We have 12 distance values, 12 angles, 1 distance sensor from other robot
// and 2 wheels actuators
// (25 + 1) * 2 = 52
const int AdvancedController::GENOME_SIZE = 54;

static CRange<Real> NN_OUTPUT_RANGE(0.0f, 1.0f);
static CRange<Real> WHEEL_ACTUATION_RANGE(-AdvancedController::MAX_VELOCITY, AdvancedController::MAX_VELOCITY);
static const CColor FOOTBOT_COLOR = CColor::GREEN;

AdvancedController::AdvancedController() {}

AdvancedController::~AdvancedController()
{
}

const Real AdvancedController::GetMaxProximityValue()
{
    const CCI_FootBotProximitySensor::TReadings &proxReads = proximity->GetReadings();
    Real maxProximity = proxReads[0].Value;
    for (size_t i = 0; i < proxReads.size(); ++i)
    {
        maxProximity = Max(maxProximity, proxReads[i].Value);
    }
    return maxProximity;
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
        distanceScannerSensor = GetSensor<CCI_FootBotDistanceScannerSensor>("footbot_distance_scanner");
        distanceScannerActuator = GetActuator<CCI_FootBotDistanceScannerActuator>("footbot_distance_scanner");

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
        distanceScannerActuator->Enable();
        distanceScannerActuator->SetRPM(ROTATION_SPEED);
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

    /* Check if there is another robot nearby */
    const CCI_RangeAndBearingSensor::TReadings &rabReads = rabSensor->GetReadings();
    Real distanceSensed = -1;
    Real angleSensed = 0;
    for (size_t i = 0; i < rabReads.size(); i++)
    {
        distanceSensed = rabReads[i].Range;
        angleSensed = rabReads[i].HorizontalBearing.GetValue();
    }

    /* Get readings from proximity sensor */
    Real maxProximity = GetMaxProximityValue();
    if (maxProximity > COLLISION_THRESHOLD)
        leds->SetAllColors(CColor::RED);
    else
        leds->SetAllColors(FOOTBOT_COLOR);

    /* Read from distance sensor; take 3 long range and 3 short range values */
    size_t i = 0;
    const CCI_FootBotDistanceScannerSensor::TReadingsMap &tDisranceReads = distanceScannerSensor->GetReadingsMap();
    for (CCI_FootBotDistanceScannerSensor::TReadingsMap::const_iterator it = tDisranceReads.begin(); it != tDisranceReads.end(); ++it)
    {
        ++it;
        perceptron.SetInput(i++, it->first.GetValue());
        if (it->second <= 0)
        {
            perceptron.SetInput(i++, it->second);
        }
        else if (it->second < SHORT_RANGE_MAX_DISTANCE)
        {
            perceptron.SetInput(i++, 1 - it->second / SHORT_RANGE_MAX_DISTANCE);
        }
        else if (it->second < LONG_RANGE_MAX_DISTANCE)
        {
            perceptron.SetInput(i++, 1 - it->second / LONG_RANGE_MAX_DISTANCE);
        }
    }

    if (useInternalSensor)
    {
        LOG << distanceSensed << std::endl;
        perceptron.SetInput(perceptron.GetNumberOfInputs() - 1, distanceSensed / 300);
        perceptron.SetInput(perceptron.GetNumberOfInputs() - 2, angleSensed);
    }
    else
    {
        perceptron.SetInput(perceptron.GetNumberOfInputs() - 1, distanceFromOpponent);
        perceptron.SetInput(perceptron.GetNumberOfInputs() - 2, angleFromOpponent);
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