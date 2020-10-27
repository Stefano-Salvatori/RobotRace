/* Include the controller definition */
#include "motor_schema.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/logging/argos_log.h>
#include <vector>
#include <utility>
#include "loop_function/utils/utils.h"
#include <string> // std::string, std::stod

#define COLLISION_THRESHOLD 0.9
#define SHORT_RANGE_MAX_DISTANCE 30
#define SHORT_RANGE_MIN_DISTANCE 4

#define LONG_RANGE_MAX_DISTANCE 150
#define LONG_RANGE_MIN_DISTANCE 20
#define ROTATION_SPEED 100
#define GO_FOREWORD_VEC_LEN  0.1
#define COURSE_VELOCITY 15

const Real MotorSchemaController::MAX_VELOCITY = 20.0f;
static const CColor FOOTBOT_COLOR = CColor::GREEN;

MotorSchemaController::MotorSchemaController() {}

MotorSchemaController::~MotorSchemaController()
{
}

const Real MotorSchemaController::GetMaxProximityValue()
{
    const CCI_FootBotProximitySensor::TReadings &proxReads = proximity->GetReadings();
    Real maxProximity = proxReads[0].Value;
    for (size_t i = 0; i < proxReads.size(); ++i)
    {
        maxProximity = Max(maxProximity, proxReads[i].Value);
    }
    return maxProximity;
}

void MotorSchemaController::Init(TConfigurationNode &t_node)
{
    /*
    * Get sensor/actuator handles
    */
    try
    {
        proximity = GetSensor<CCI_FootBotProximitySensor>("footbot_proximity");
        positioning = GetSensor<CCI_PositioningSensor>("positioning");
        leds = GetActuator<CCI_LEDsActuator>("leds");
        wheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
        wheelsSensor = GetSensor<CCI_DifferentialSteeringSensor>("differential_steering");
        distanceScannerSensor = GetSensor<CCI_FootBotDistanceScannerSensor>("footbot_distance_scanner");
        distanceScannerActuator = GetActuator<CCI_FootBotDistanceScannerActuator>("footbot_distance_scanner");

    }
    catch (CARGoSException &ex)
    {
        THROW_ARGOSEXCEPTION_NESTED("Error initializing sensors/actuators", ex);
    }

    try
    {
        distanceScannerActuator->Enable();
        distanceScannerActuator->SetRPM(ROTATION_SPEED);
        leds->SetAllColors(FOOTBOT_COLOR);
    }
    catch (CARGoSException &ex)
    {
        THROW_ARGOSEXCEPTION_NESTED("Error initializing the perceptron network", ex);
    }
}

CRadians MotorSchemaController::GetRobotAngle()
{
    std::stringstream ss;
    ss << positioning->GetReading().Orientation << std::endl;
    std::string s = ss.str();
    std::string s2 = s.substr(0, s.find(","));
    CRadians c(0);
    c.FromValueInDegrees(std::stod(s, 0));
     return c;
}

CVector2 MotorSchemaController::StayOnPath()
{
    const CCI_FootBotDistanceScannerSensor::TReadingsMap &tLongDistanceReads = distanceScannerSensor->GetLongReadingsMap();
    const CCI_FootBotDistanceScannerSensor::TReadingsMap &tShortDistanceReads = distanceScannerSensor->GetShortReadingsMap();

    std::vector<CVector2> forces;
    for (CCI_FootBotDistanceScannerSensor::TReadingsMap::const_iterator it = tLongDistanceReads.begin(); it != tLongDistanceReads.end(); ++it)
    {
        Real d = it->second;
        CVector2 v(0,0);
        v.FromPolarCoordinates(1, CRadians::ZERO);

        if (d != -1 && d != -2 and d <= LONG_RANGE_MAX_DISTANCE && d > LONG_RANGE_MIN_DISTANCE)
        {
            Real length = ((LONG_RANGE_MAX_DISTANCE - d) / LONG_RANGE_MAX_DISTANCE);
            CRadians angle = -Sign(it->first) * CRadians::PI_OVER_TWO;
            v.FromPolarCoordinates(length, angle);
        }
        forces.push_back(v);
    }

    for (CCI_FootBotDistanceScannerSensor::TReadingsMap::const_iterator it = tShortDistanceReads.begin(); it != tShortDistanceReads.end(); ++it)
    {
        Real d = it->second;
        CVector2 v(0,0);
        v.FromPolarCoordinates(1, CRadians::ZERO);
        if (d != -1 && d != -2 and d <= SHORT_RANGE_MAX_DISTANCE && d > SHORT_RANGE_MIN_DISTANCE)
        {
            Real length = ((SHORT_RANGE_MAX_DISTANCE - d) / SHORT_RANGE_MAX_DISTANCE);
            CRadians angle = - Sign(it->first) * CRadians::PI_OVER_TWO;
            v.FromPolarCoordinates(length, angle);
        }
        forces.push_back(v);
    }
    CVector2 stayOnPath = Vec2Summation(forces);
    stayOnPath.FromPolarCoordinates(stayOnPath.Length() / forces.size(), stayOnPath.Angle());
    return stayOnPath;
}

CVector2 MotorSchemaController::AvoidObstacoles()
{
    CCI_FootBotProximitySensor::TReadings proxReads = proximity->GetReadings();
    std::vector<CVector2> forces;
    for (size_t i = 0; i < proxReads.size(); i++)
    {
        CVector2 v(0,0);
        v.FromPolarCoordinates(proxReads[i].Value, CRadians::PI_OVER_TWO + proxReads[i].Angle);
        forces.push_back(v);
    }

    CVector2 avoidObs = Vec2Summation(forces);
    avoidObs.FromPolarCoordinates(avoidObs.Length() / forces.size(), avoidObs.Angle());
    return avoidObs;
}

CVector2 MotorSchemaController::Vec2Summation(std::vector<CVector2> vecArray)
{
    CVector2 res(0,0);
    for(size_t i=0; i<vecArray.size(); i++){
        res += vecArray[i];
    }
    return res;
}

void MotorSchemaController::ControlStep()
{

    /* Get readings from proximity sensor */
    Real maxProximity = GetMaxProximityValue();
    if (maxProximity > COLLISION_THRESHOLD)
        leds->SetAllColors(CColor::RED);
    else
        leds->SetAllColors(FOOTBOT_COLOR);



    CRadians robotAngle = GetRobotAngle();
   
    CVector2 stay_on_path =  StayOnPath();
    
    CVector2 avoid_obstacoles = AvoidObstacoles();
   
    CVector2 go_foreward(0,0);
    go_foreward.FromPolarCoordinates(GO_FOREWORD_VEC_LEN, -CRadians::PI_OVER_TWO - robotAngle);
    
    // compute result vector
    std::vector<CVector2> schemas = {stay_on_path, avoid_obstacoles, go_foreward};
    CVector2 resultant = Vec2Summation(schemas);
    resultant.FromPolarCoordinates(resultant.Length() / schemas.size(), resultant.Angle());

    // transform to differential model
    Real half_l = wheelsSensor->GetReading().WheelAxisLength / 2;
    Real left_v = resultant.Length() - (half_l * resultant.Angle().GetValue());
    Real right_v = resultant.Length() + (half_l * resultant.Angle().GetValue());


    Real leftSpeed = Map(left_v, -1, 1, COURSE_VELOCITY, MAX_VELOCITY);
    Real rightSpeed = Map(right_v, -1, 1, COURSE_VELOCITY, MAX_VELOCITY);

     wheels->SetLinearVelocity(leftSpeed, rightSpeed);
}

void MotorSchemaController::Reset()
{
    distanceScannerActuator->Enable();
    distanceScannerActuator->SetRPM(ROTATION_SPEED);
    leds->SetAllColors(FOOTBOT_COLOR);
}

void MotorSchemaController::Destroy()
{
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
REGISTER_CONTROLLER(MotorSchemaController, "motor_schema_controller")