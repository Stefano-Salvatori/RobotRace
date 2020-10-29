#include "race_loop_function.h"
#include <sstream>

// The experiment finishes when the robot is less then MIN_DISTANCE_FROM_FINISH distant from the finish line
#define MIN_DISTANCE_FROM_FINISH 0.05

bool DEBUG = false;
const CVector3 OBSTACLE_POSITIONS[NUM_OBSTACLES] = {
    CVector3(0, 5, 0),
    CVector3(1, 4, 0),
    CVector3(0, 1, 0),
    CVector3(-1, 0, 0),
    CVector3(-1, -4, 0),
    CVector3(1, -5, 0),
};

RaceLoopFunction::RaceLoopFunction() : m_pcRNG(NULL),
                                       stepCount(0)
{
}

RaceLoopFunction::~RaceLoopFunction()
{
}

void RaceLoopFunction ::RemoveObstacles()
{
    for (size_t i = 0; i < NUM_OBSTACLES; i++)
    {
        RemoveEntity(*obstacles[i]);
    }
}

void RaceLoopFunction ::AddObstacles()
{
    CRange<Real> positionRange = CRange<Real>(-1.0, 1.0);
    CRange<Real> dimensionRange = CRange<Real>(obstaclesMinSize, obstaclesMaxSize);
    CRange<CRadians> angleRange = CRange<CRadians>(-CRadians::PI_OVER_THREE, CRadians::PI_OVER_THREE);
    for (size_t i = 0; i < NUM_OBSTACLES; i++)
    {
        const CVector3 basePosition = OBSTACLE_POSITIONS[i];
        obstacles[i] = new CBoxEntity(
            "o" + ToString(i),
            CVector3(basePosition.GetX() + m_pcRNG->Uniform(positionRange), basePosition.GetY(), basePosition.GetZ()),
            CQuaternion().FromEulerAngles(m_pcRNG->Uniform(angleRange), CRadians::ZERO, CRadians::ZERO),
            false,
            CVector3(m_pcRNG->Uniform(dimensionRange), 0.1, 0.5));
        AddEntity(*obstacles[i]);
    }
}

void RaceLoopFunction::Init(TConfigurationNode &t_node)
{
    try
    {
        m_pcRNG = CRandom::CreateRNG("argos");

        GetNodeAttributeOrDefault(t_node, "numBots", numBots, DEFAULT_NUM_BOTS);
        std::string controllersNames;
        GetNodeAttributeOrDefault(t_node, "controllers", controllersNames, std::string(""));

        //controller names as comma separated string
        std::vector<std::string> controllersParsed;
        std::stringstream s_stream(controllersNames); //create string stream from the string
        while (s_stream.good())
        {
            std::string substr;
            //get first string delimited by comma
            getline(s_stream, substr, ',');
            //remove spaces
            substr.erase(std::remove_if(substr.begin(), substr.end(), isspace), substr.end());
            controllersParsed.push_back(substr);
        }

        // create footbots (each footbot as a different controller)
        for (size_t botsIndex = 0; botsIndex < numBots; botsIndex++)
        {
            const std::string controllerName = controllersParsed[botsIndex % controllersParsed.size()];
            CFootBotEntity *bot = new CFootBotEntity(
                controllerName + ToString(botsIndex), // entity id
                controllerName                        // controller id as set in the XML
            );
            bots.push_back(*bot);
            AddEntity(*bot);
        }

        /* Add Random obstacles in the map */
        AddObstacles();

        GetNodeAttributeOrDefault(t_node, "debug", DEBUG, false);

        // Get finish line from configuration file
        GetNodeAttributeOrDefault(t_node, "finishSegmentV1", finishSegmentV1, CVector2(0, 0));
        GetNodeAttributeOrDefault(t_node, "finishSegmentV2", finishSegmentV2, CVector2(0, 0));

        //Get starting position from configuration file
        GetNodeAttributeOrDefault(t_node, "startingSegmentV1", startingSegmentV1, CVector2(0, 0));
        GetNodeAttributeOrDefault(t_node, "startingSegmentV2", startingSegmentV2, CVector2(0, 0));

        //Get obsacles dimensions
        GetNodeAttributeOrDefault(t_node, "obstaclesMinSize", obstaclesMinSize, DEFAULT_OBSTACLE_MIN_SIZE);
        GetNodeAttributeOrDefault(t_node, "obstaclesMaxSize", obstaclesMaxSize, DEFAULT_OBSTACLE_MAX_SIZE);

        //Initialize footbot positions
        Reset();
    }
    catch (CARGoSException &ex)
    {
    }
}

void RaceLoopFunction::PreStep()
{
}

void RaceLoopFunction::PostStep()
{
    stepCount++;
}

bool RaceLoopFunction ::IsExperimentFinished()
{
    for (size_t i = 0; i < bots.size(); i++)
    {
        const CVector2 pos = GetFootBotPosition(i);
        if (DistanceFromSegment(pos, finishSegmentV1, finishSegmentV2) < MIN_DISTANCE_FROM_FINISH)
        {
            if (DEBUG)
            {
                LOG << "winner: " << bots[i].GetControllableEntity().GetController().GetId() << std ::endl;
                PrintNumCollisions(LOG.GetStream());
            }
            return true;
        }
    }
    return false;
}

void RaceLoopFunction ::Reset()
{
    stepCount = 0;
    RemoveObstacles();
    AddObstacles();
    for (size_t i = 0; i < bots.size(); i++)
    {
        // Move robot to the initial position
        const CVector2 pos = SubdivideSegment(startingSegmentV1, startingSegmentV2, (i * 2 + 1) / (bots.size() * 2.0));
        CQuaternion orientation;
        orientation = orientation.FromEulerAngles(-CRadians::PI_OVER_TWO, CRadians::ZERO, CRadians::ZERO);
        if (!MoveEntity(
                bots[i].GetEmbodiedEntity(),         // move the body of the robot
                CVector3(pos.GetX(), pos.GetY(), 0), // to osition
                orientation,
                false))
        {
            LOGERR << "Can't move robot " << i << "  in starting position" << std::endl;
        }
    }
}

std::string RaceLoopFunction::Winner()
{
    for (size_t i = 0; i < bots.size(); i++)
    {
        const CVector2 pos = GetFootBotPosition(i);
        if (DistanceFromSegment(pos, finishSegmentV1, finishSegmentV2) < MIN_DISTANCE_FROM_FINISH)
        {
            return bots[i].GetControllableEntity().GetController().GetId();
        }
    }
    return "tie";
}

REGISTER_LOOP_FUNCTIONS(RaceLoopFunction, "race_loop_function");
