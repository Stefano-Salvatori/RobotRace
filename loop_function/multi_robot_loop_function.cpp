#include "multi_robot_loop_function.h"

#define SAMPLE_STEP_VALUE 10

// The experiment finishes when the robot is less then MIN_DISTANCE_FROM_FINISH distant from the finish line
#define MIN_DISTANCE_FROM_FINISH 0.01

MultiRobotLoopFunction::MultiRobotLoopFunction() : stepCount(0),
                                                   totalStepProximity(0),
                                                   totalStepRightWheelSpeed(0),
                                                   totalStepLeftWheelSpeed(0),
                                                   stepByStepPerformance(0),
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
    const Real robotsDistance = Distance(GetFootbotPosition(mainFootbot), GetFootbotPosition(opponentFootbot));
    mainController->SetDistanceFromOpponent(robotsDistance);
    opponentController->SetDistanceFromOpponent(robotsDistance);

    CCI_FootBotProximitySensor::TReadings proximityValues = mainController->GetProximityReadings();
    Real maxProximity = proximityValues[0].Value;
    for (size_t i = 0; i < proximityValues.size(); ++i)
    {
        maxProximity = Max(maxProximity, proximityValues[i].Value);
    }

    this->totalStepProximity += maxProximity;
    this->totalStepRightWheelSpeed += this->mainController->GetRightSpeed();
    this->totalStepLeftWheelSpeed += this->mainController->GetLeftSpeed();

    if (this->stepCount % SAMPLE_STEP_VALUE == 0)
    {
        const Real avoidCollisions = 1 - this->totalStepProximity / SAMPLE_STEP_VALUE;
        const Real avgRightSpeed = this->totalStepRightWheelSpeed / (SAMPLE_STEP_VALUE * AdvancedController::MAX_VELOCITY);
        const Real avgLeftSpeed = this->totalStepLeftWheelSpeed / (SAMPLE_STEP_VALUE * AdvancedController::MAX_VELOCITY);
        const Real goFast = (abs(avgRightSpeed) + abs(avgLeftSpeed)) / 2;
        const Real goStraight = 1 - sqrt(abs(avgRightSpeed - avgLeftSpeed) / 2);
        stepByStepPerformance += avoidCollisions * goFast;

        this->totalStepProximity = 0;
        this->totalStepRightWheelSpeed = 0;
        this->totalStepLeftWheelSpeed = 0;
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
    if (currentGeneration == 0)
    {
        opponentController->GetPerceptron().SetOnlineParameters(opponentController->GENOME_SIZE, controllerParameters);
    }
    else
    {
        opponentController->GetPerceptron().LoadNetworkParameters("best/best_" + ToString(currentGeneration) + ".dat");
    }
}

/****************************************/
/****************************************/

Real MultiRobotLoopFunction::Performance()
{
    size_t summedValuesCount = this->stepCount / SAMPLE_STEP_VALUE;
    const Real distanceFromFinish = DistanceFromSegment(GetFootbotPosition(mainFootbot), this->finishSegmentV1, this->finishSegmentV2);
    return stepByStepPerformance / (summedValuesCount * distanceFromFinish * this->stepCount);
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(MultiRobotLoopFunction, "multi_robot_loop_function");
