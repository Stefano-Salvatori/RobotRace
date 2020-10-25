#include "multi_robot_loop_function.h"

#define SEGMENT_LENGTH 10

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

MultiRobotLoopFunction::MultiRobotLoopFunction() : stepCount(0),
                                                   totalStepProximity(0),
                                                   totalStepRightWheelSpeed(0),
                                                   totalStepLeftWheelSpeed(0),
                                                   totalOnSegmentPerformance(0),
                                                   mainFootbot(NULL),
                                                   mainController(NULL),
                                                   opponentFootbot(NULL),
                                                   opponentController(NULL),
                                                   controllerParameters(new Real[AdvancedController::GENOME_SIZE]),
                                                   m_pcRNG(NULL)
{
}

/****************************************/
/****************************************/

MultiRobotLoopFunction::~MultiRobotLoopFunction()
{
    delete[] controllerParameters;
}

void MultiRobotLoopFunction::RemoveObstacles()
{
    for (size_t i = 0; i < NUM_OBSTACLES; i++)
    {
        RemoveEntity(*this->obstacles[i]);
    }
}

void MultiRobotLoopFunction::AddObstacles()
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

void MultiRobotLoopFunction::Init(TConfigurationNode &t_node)
{
    /*
    * Create the random number generator
    */
    m_pcRNG = CRandom::CreateRNG("argos");

    mainFootbot = new CFootBotEntity(
        "fb",        // entity id
        "genetic_nn" // controller id as set in the XML
    );
    AddEntity(*mainFootbot);
    mainController = &dynamic_cast<AdvancedController &>(mainFootbot->GetControllableEntity().GetController());

    opponentFootbot = new CFootBotEntity(
        "opponent_fb", // entity id
        "genetic_nn"   // controller id as set in the XML
    );
    AddEntity(*opponentFootbot);
    opponentController = &dynamic_cast<AdvancedController &>(opponentFootbot->GetControllableEntity().GetController());

    /* Add Random obstacles in the map */
    AddObstacles();
    /*
    * Process trial information, if any
    */
    try
    {
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

void MultiRobotLoopFunction::PreStep()
{
}

void MultiRobotLoopFunction::PostStep()
{
    this->stepCount++;
    const Real robotsDistance = 1 - Distance(GetFootbotPosition(mainFootbot), GetFootbotPosition(opponentFootbot)) / 4;
    const Real robotsAngle = AngleBetweenPoints(GetFootbotPosition(mainFootbot), GetFootbotPosition(opponentFootbot));
    mainController->SetDistanceFromOpponent(robotsDistance);
    opponentController->SetDistanceFromOpponent(robotsDistance);
    mainController->SetAngleFromOpponent(robotsAngle);
    opponentController->SetAngleFromOpponent(robotsAngle);

    Real maxProximity = mainController->GetMaxProximityValue();

    this->totalStepProximity += maxProximity;
    this->totalStepRightWheelSpeed += this->mainController->GetRightSpeed();
    this->totalStepLeftWheelSpeed += this->mainController->GetLeftSpeed();

    if (this->stepCount % SEGMENT_LENGTH == 0)
    {
        const Real avoidCollisions = 1 - this->totalStepProximity / SEGMENT_LENGTH;
        //velocity scaled to [-1, 1]
        const Real avgRightSpeed = (this->totalStepRightWheelSpeed / SEGMENT_LENGTH) / AdvancedController::MAX_VELOCITY;
        const Real avgLeftSpeed = (this->totalStepLeftWheelSpeed / SEGMENT_LENGTH) / AdvancedController::MAX_VELOCITY;
        const Real goStraight = 1 / (1 + abs(avgRightSpeed - avgLeftSpeed));
        const Real goFast = abs((avgRightSpeed + avgLeftSpeed) / 2);

        totalOnSegmentPerformance += avoidCollisions * avoidCollisions * goFast * goStraight;

        this->totalStepProximity = 0;
        this->totalStepRightWheelSpeed = 0;
        this->totalStepLeftWheelSpeed = 0;
        if (DEBUG)
        {
            size_t segmentsAnalyzed = this->stepCount / SEGMENT_LENGTH;
            LOG << this->stepCount << std::endl;
            const Real dist = DistanceFromSegment(GetFootbotPosition(mainFootbot), this->finishSegmentV1, this->finishSegmentV2);
            const Real reachFinishLine = 1 / (1 + dist);
            LOG << "dist: " << reachFinishLine << std::endl;
            LOG << "Avg segment performance after " << segmentsAnalyzed << " segments: "
                << totalOnSegmentPerformance / segmentsAnalyzed
                << std::endl;
        }
    }
}

bool MultiRobotLoopFunction::IsExperimentFinished()
{
    DistanceFromSegment(GetFootbotPosition(mainFootbot), this->finishSegmentV1, this->finishSegmentV2) < MIN_DISTANCE_FROM_FINISH;
}

/****************************************/
/****************************************/

void MultiRobotLoopFunction::Reset()
{
    this->stepCount = 0;
    this->totalStepProximity = 0;
    this->totalStepRightWheelSpeed = 0;
    this->totalStepLeftWheelSpeed = 0;
    this->totalOnSegmentPerformance = 0;

    this->RemoveObstacles();
    this->AddObstacles();

    // Move robot to the initial position
    const CVector2 mainFootbotPos = SubdivideSegment(startingSegmentV1, startingSegmentV2, 1.0 / 4);
    const CVector2 opponentFootbotPos = SubdivideSegment(startingSegmentV1, startingSegmentV2, 3.0 / 4);
    CQuaternion orientation;
    orientation = orientation.FromEulerAngles(CRadians::ZERO, CRadians::ZERO, CRadians::ZERO);
    if (!MoveEntity(
            mainFootbot->GetEmbodiedEntity(),                          // move the body of the robot
            CVector3(mainFootbotPos.GetX(), mainFootbotPos.GetY(), 0), // to this position
            orientation,
            false) ||
        !MoveEntity(
            opponentFootbot->GetEmbodiedEntity(),                              // move the body of the robot
            CVector3(opponentFootbotPos.GetX(), opponentFootbotPos.GetY(), 0), // to this position
            orientation,
            false))
    {
        LOGERR << "Can't move robot  in starting position" << std::endl;
    }
}

/****************************************/
/****************************************/

void MultiRobotLoopFunction::ConfigureFromGenome(const GARealGenome &c_genome)
{
    /* Copy the genes into the NN parameter buffer */
    for (size_t i = 0; i < mainController->GENOME_SIZE; ++i)
    {
        controllerParameters[i] = c_genome[i];
    }
    /* Set the NN parameters */
    mainController->GetPerceptron().SetOnlineParameters(mainController->GENOME_SIZE, controllerParameters);
    if (currentGeneration <= 10)
    {
        opponentController->GetPerceptron().SetOnlineParameters(opponentController->GENOME_SIZE, controllerParameters);
    }
    else
    {
        opponentController->GetPerceptron().LoadNetworkParameters("best_multiple/best_" + ToString((currentGeneration / 10) * 10) + ".dat");
    }
}

/****************************************/
/****************************************/

Real MultiRobotLoopFunction::Performance()
{
    size_t segmentsAnalyzed = this->stepCount / SEGMENT_LENGTH;
    const Real avgOnSegmentPerformance = totalOnSegmentPerformance / segmentsAnalyzed;
    const Real dist = DistanceFromSegment(GetFootbotPosition(mainFootbot), this->finishSegmentV1, this->finishSegmentV2);
    const Real reachFinishLine = 1 / (1 + dist);
    return avgOnSegmentPerformance * reachFinishLine;
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(MultiRobotLoopFunction, "multi_robot_loop_function");
