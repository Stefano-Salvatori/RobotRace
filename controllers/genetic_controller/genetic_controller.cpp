/* Include the controller definition */
#include "genetic_controller.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/logging/argos_log.h>

#define SHORT_RANGE_MAX_DISTANCE 30
#define LONG_RANGE_MAX_DISTANCE 150
#define ROTATION_SPEED 50

// Formula for number of parameters:
//  (Input + 1) * Output
// We have 12 distance values, 12 angles and 2 wheels actuators
//  (24 + 1) * 2 = 50
const int GeneticController::GENOME_SIZE = 50;

static CRange<Real> NN_OUTPUT_RANGE(0.0f, 1.0f);
static CRange<Real> WHEEL_ACTUATION_RANGE(-CommonController::MAX_VELOCITY, CommonController::MAX_VELOCITY);
static const CColor FOOTBOT_COLOR = CColor::GREEN;

GeneticController::GeneticController() {}

GeneticController::~GeneticController()
{
}

void GeneticController::Init(TConfigurationNode &t_node)
{
    /*
    * Get sensor/actuator handles
    */
    try
    {
        proximity = GetSensor<CCI_FootBotProximitySensor>("footbot_proximity");
        leds = GetActuator<CCI_LEDsActuator>("leds");
        wheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
        distanceScannerSensor = GetSensor<CCI_FootBotDistanceScannerSensor>("footbot_distance_scanner");
        distanceScannerActuator = GetActuator<CCI_FootBotDistanceScannerActuator>("footbot_distance_scanner");
    }
    catch (CARGoSException &ex)
    {
        THROW_ARGOSEXCEPTION_NESTED("Error initializing sensors/actuators", ex);
    }

    try
    {
        perceptron.Init(t_node);
        distanceScannerActuator->Enable();
        distanceScannerActuator->SetRPM(ROTATION_SPEED);
        leds->SetAllColors(FOOTBOT_COLOR);
    }
    catch (CARGoSException &ex)
    {
        THROW_ARGOSEXCEPTION_NESTED("Error initializing the perceptron network", ex);
    }
}

void GeneticController::ControlStep()
{
    /* Get readings from proximity sensor */
    Real maxProximity = GetMaxProximityValue();
    if (maxProximity > CommonController::COLLIISION_THRESHOLD)
    {
        numCollisions++;
        leds->SetAllColors(CColor::RED);
    }
    else
        leds->SetAllColors(FOOTBOT_COLOR);

    /* Read from distance sensor; take 6 long range and 6 short range values */
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

void GeneticController::Reset()
{
    numCollisions = 0;
    perceptron.Reset();
    distanceScannerActuator->Enable();
    distanceScannerActuator->SetRPM(ROTATION_SPEED);
    leds->SetAllColors(FOOTBOT_COLOR);
}

void GeneticController::Destroy()
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
REGISTER_CONTROLLER(GeneticController, "genetic_controller")