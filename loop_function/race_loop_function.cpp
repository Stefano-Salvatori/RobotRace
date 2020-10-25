#include "race_loop_function.h"

// The experiment finishes when the robot is less then MIN_DISTANCE_FROM_FINISH distant from the finish line
#define MIN_DISTANCE_FROM_FINISH 0.01

bool DEBUG = false;
const CVector3 OBSTACLE_POSITIONS[NUM_OBSTACLES] = {
    CVector3(0, 5, 0),
    CVector3(1, 4, 0),
    CVector3(0, 1, 0),
    CVector3(-1, 0, 0),
    CVector3(-1, -4, 0),
    CVector3(1, -5, 0),
};

RaceLoopFunction::RaceLoopFunction() : m_pcRNG(NULL)
{
}

RaceLoopFunction::~RaceLoopFunction()
{
}

void RaceLoopFunction ::RemoveObstacles()
{
    for (size_t i = 0; i < NUM_OBSTACLES; i++)
    {
        RemoveEntity(*this->obstacles[i]);
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
        this->obstacles[i] = new CBoxEntity(
            "o" + ToString(i),
            CVector3(basePosition.GetX() + m_pcRNG->Uniform(positionRange), basePosition.GetY(), basePosition.GetZ()),
            CQuaternion().FromEulerAngles(m_pcRNG->Uniform(angleRange), CRadians::ZERO, CRadians::ZERO),
            false,
            CVector3(m_pcRNG->Uniform(dimensionRange), 0.1, 0.5));
        AddEntity(*this->obstacles[i]);
    }
}

/****************************************/
/****************************************/

void RaceLoopFunction ::Init(TConfigurationNode &t_node)
{
    /*
    * Create the random number generator
    */
    m_pcRNG = CRandom::CreateRNG("argos");

    GetNodeAttributeOrDefault(t_node, "numBots", numBots, DEFAULT_NUM_BOTS);
    bots.reserve(numBots);
    controllers.reserve(numBots);

    for (size_t botsIndex = 0; botsIndex < numBots; botsIndex++)
    {
        CFootBotEntity *bot = new CFootBotEntity(
            "fb" + ToString(botsIndex), // entity id
            "genetic_nn"                // controller id as set in the XML
        );
        bots.push_back(*bot);
        AddEntity(*bot);
        controllers.push_back(dynamic_cast<GeneticController &>(bot->GetControllableEntity().GetController()));
    }

    /* Add Random obstacles in the map */
    AddObstacles();

    /*
    * Process trial information, if any
    */
    try
    {
        GetNodeAttributeOrDefault(t_node, "debug", DEBUG, false);

        // Get finish line from configuration file
        GetNodeAttributeOrDefault(t_node, "finishSegmentV1", this->finishSegmentV1, CVector2(0, 0));
        GetNodeAttributeOrDefault(t_node, "finishSegmentV2", this->finishSegmentV2, CVector2(0, 0));

        //Get starting position from configuration file
        GetNodeAttributeOrDefault(t_node, "startingSegmentV1", this->startingSegmentV1, CVector2(0, 0));
        GetNodeAttributeOrDefault(t_node, "startingSegmentV2", this->startingSegmentV2, CVector2(0, 0));

        //Get obsacles dimensions
        GetNodeAttributeOrDefault(t_node, "obstaclesMinSize", this->obstaclesMinSize, DEFAULT_OBSTACLE_MIN_SIZE);
        GetNodeAttributeOrDefault(t_node, "obstaclesMaxSize", this->obstaclesMaxSize, DEFAULT_OBSTACLE_MAX_SIZE);

        Reset();
    }
    catch (CARGoSException &ex)
    {
    }
}

void RaceLoopFunction ::PreStep()
{
    LOG << "Ciao PreStep" << std::endl;
}

void RaceLoopFunction ::PostStep()
{
    LOG << "Ciao PostStep" << std::endl;
}

bool RaceLoopFunction ::IsExperimentFinished()
{
    for (size_t i = 0; i < bots.size(); i++)
    {
        LOG << "Ciao \n";
        if (DistanceFromSegment(GetFootBotPosition(i), this->finishSegmentV1, this->finishSegmentV2) < MIN_DISTANCE_FROM_FINISH)
        {
            return true;
        }
    }
    return false;
}

void RaceLoopFunction ::Reset()
{
    this->RemoveObstacles();
    this->AddObstacles();
    for (size_t i = 0; i < bots.size(); i++)
    {
        // Move robot to the initial position
        const CVector2 pos = SubdivideSegment(startingSegmentV1, startingSegmentV2, (i * 2 + 1) / (bots.size() * 2.0));
        CQuaternion orientation;
        orientation = orientation.FromEulerAngles(CRadians::ZERO, CRadians::ZERO, CRadians::ZERO);
        if (!MoveEntity(
                bots[i].GetEmbodiedEntity(),         // move the body of the robot
                CVector3(pos.GetX(), pos.GetY(), 0), // to this position
                orientation,
                false))
        {
            LOGERR << "Can't move robot " << i << "  in starting position" << std::endl;
        }
    }
}

/****************************************/
/****************************************/

Real RaceLoopFunction ::Performance()
{
    /*size_t segmentsAnalyzed = this->stepCount / SEGMENT_LENGTH;
    const Real avgOnSegmentPerformance = totalOnSegmentPerformance / segmentsAnalyzed;
    const Real dist = DistanceFromSegment(GetFootBotPosition(), this->finishSegmentV1, this->finishSegmentV2);
    const Real reachFinishLine = 1 / (1 + dist);
    return avgOnSegmentPerformance * reachFinishLine;*/
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(RaceLoopFunction, "race_loop_function");