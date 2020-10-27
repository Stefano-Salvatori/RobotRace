#include "single_robot_loop_function.h"

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

SingleRobotLoopFunction::SingleRobotLoopFunction() : stepCount(0),
                                                     totalStepProximity(0),
                                                     totalStepRightWheelSpeed(0),
                                                     totalStepLeftWheelSpeed(0),
                                                     totalOnSegmentPerformance(0),
                                                     footBot(NULL),
                                                     controller(NULL),
                                                     parameters(new Real[GeneticController::GENOME_SIZE]),
                                                     m_pcRNG(NULL)
{
}

SingleRobotLoopFunction::~SingleRobotLoopFunction()
{
   delete[] parameters;
}

void SingleRobotLoopFunction::RemoveObstacles()
{
   for (size_t i = 0; i < NUM_OBSTACLES; i++)
   {
      RemoveEntity(*this->obstacles[i]);
   }
}

void SingleRobotLoopFunction::AddObstacles()
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

void SingleRobotLoopFunction::Init(TConfigurationNode &t_node)
{
   /*
    * Create the random number generator
    */
   m_pcRNG = CRandom::CreateRNG("argos");

   /*
    * Create the foot-bot and get a reference to its controller
    */
   footBot = new CFootBotEntity(
       "fb",        // entity id
       "genetic_nn" // controller id as set in the XML
   );
   AddEntity(*footBot);
   controller = &dynamic_cast<CommonController &>(footBot->GetControllableEntity().GetController());

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

void SingleRobotLoopFunction::PreStep()
{
}

void SingleRobotLoopFunction::PostStep()
{
   this->stepCount++;
   Real maxProximity = controller->GetMaxProximityValue();

   this->totalStepProximity += maxProximity;
   this->totalStepRightWheelSpeed += this->controller->GetRightSpeed();
   this->totalStepLeftWheelSpeed += this->controller->GetLeftSpeed();

   if (this->stepCount % SEGMENT_LENGTH == 0)
   {
      const Real avoidCollisions = 1 - this->totalStepProximity / SEGMENT_LENGTH;

      //velocity scaled to [-1, 1]
      const Real avgRightSpeed = (this->totalStepRightWheelSpeed / SEGMENT_LENGTH) / CommonController::MAX_VELOCITY;
      const Real avgLeftSpeed = (this->totalStepLeftWheelSpeed / SEGMENT_LENGTH) / CommonController::MAX_VELOCITY;
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
         const Real dist = DistanceFromSegment(GetFootBotPosition(), this->finishSegmentV1, this->finishSegmentV2);
         const Real reachFinishLine = 1 / (1 + dist);
         LOG << "dist: " << reachFinishLine << std::endl;
         LOG << "Avg segment performance after " << segmentsAnalyzed << " segments: "
             << totalOnSegmentPerformance / segmentsAnalyzed
             << std::endl;
      }
   }
}

bool SingleRobotLoopFunction::IsExperimentFinished()
{
   return DistanceFromSegment(GetFootBotPosition(), this->finishSegmentV1, this->finishSegmentV2) < MIN_DISTANCE_FROM_FINISH;
}

void SingleRobotLoopFunction::Reset()
{
   this->stepCount = 0;
   this->totalStepProximity = 0;
   this->totalStepRightWheelSpeed = 0;
   this->totalStepLeftWheelSpeed = 0;
   this->totalOnSegmentPerformance = 0;

   const CVector3 startingPosition = GetRandomStartingPosition();

   this->RemoveObstacles();
   this->AddObstacles();
   /*
    * Move robot to the initial position 
    */
   if (!MoveEntity(
           footBot->GetEmbodiedEntity(),
           startingPosition,
           CQuaternion().FromEulerAngles(-CRadians::PI_OVER_TWO, CRadians::ZERO, CRadians::ZERO),
           false))
   {
      LOGERR << "Can't move robot in <" << startingPosition << ">" << std::endl;
   }
}

/****************************************/
/****************************************/

void SingleRobotLoopFunction::ConfigureFromGenome(const GARealGenome &c_genome)
{
   GeneticController *geneticController = dynamic_cast<GeneticController*>(controller);
   /* Copy the genes into the NN parameter buffer */
   for (size_t i = 0; i < geneticController->GENOME_SIZE; ++i)
   {
      parameters[i] = c_genome[i];
   }
   /* Set the NN parameters */
   geneticController->GetPerceptron().SetOnlineParameters(geneticController->GENOME_SIZE, parameters);
}

/****************************************/
/****************************************/

Real SingleRobotLoopFunction::Performance()
{
   size_t segmentsAnalyzed = this->stepCount / SEGMENT_LENGTH;
   const Real avgOnSegmentPerformance = totalOnSegmentPerformance / segmentsAnalyzed;
   const Real dist = DistanceFromSegment(GetFootBotPosition(), this->finishSegmentV1, this->finishSegmentV2);
   const Real reachFinishLine = 1 / (1 + dist);
   return avgOnSegmentPerformance * reachFinishLine;
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(SingleRobotLoopFunction, "single_robot_loop_function");
