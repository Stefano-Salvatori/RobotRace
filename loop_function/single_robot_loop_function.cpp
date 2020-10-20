#include "single_robot_loop_function.h"

#define SEGMENT_LENGTH 10

// The experiment finishes when the robot is less then MIN_DISTANCE_FROM_FINISH distant from the finish line
#define MIN_DISTANCE_FROM_FINISH 0.01

bool DEBUG = false;

SingleRobotLoopFunction::SingleRobotLoopFunction() : stepCount(0),
                                                     totalStepProximity(0),
                                                     totalStepRightWheelSpeed(0),
                                                     totalStepLeftWheelSpeed(0),
                                                     totalOnSegmentPerformance(0),
                                                     footBot(NULL),
                                                     controller(NULL),
                                                     parameters(new Real[SimpleController::GENOME_SIZE]),
                                                     m_pcRNG(NULL)
{
}

/****************************************/
/****************************************/

SingleRobotLoopFunction::~SingleRobotLoopFunction()
{
   delete[] parameters;
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
   controller = &dynamic_cast<SimpleController &>(footBot->GetControllableEntity().GetController());

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
   CCI_FootBotProximitySensor::TReadings proximityValues = controller->GetProximityReadings();
   Real maxProximity = proximityValues[0].Value;
   for (size_t i = 0; i < proximityValues.size(); ++i)
   {
      maxProximity = Max(maxProximity, proximityValues[i].Value);
   }

   this->totalStepProximity += maxProximity;
   this->totalStepRightWheelSpeed += this->controller->GetRightSpeed();
   this->totalStepLeftWheelSpeed += this->controller->GetLeftSpeed();

   if (this->stepCount % SEGMENT_LENGTH == 0)
   {
      const Real avoidCollisions = 1 - this->totalStepProximity / SEGMENT_LENGTH;
      const Real avgRightSpeed = this->totalStepRightWheelSpeed / SEGMENT_LENGTH;
      const Real avgLeftSpeed = this->totalStepLeftWheelSpeed / SEGMENT_LENGTH;

      //velocity scaled to 0 - 1
      const Real goFast = abs((avgRightSpeed + avgLeftSpeed) / (2 * SimpleController::MAX_VELOCITY));
      // const Real goStraight = 1 - sqrt(abs(avgRightSpeed - avgLeftSpeed) / 2);

      totalOnSegmentPerformance += avoidCollisions * goFast;

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

         LOG << "Avg segment performance after " << segmentsAnalyzed << "segments: "
             << totalOnSegmentPerformance / segmentsAnalyzed
             << std::endl;
      }
   }
}

bool SingleRobotLoopFunction::IsExperimentFinished()
{
   return DistanceFromSegment(GetFootBotPosition(), this->finishSegmentV1, this->finishSegmentV2) < MIN_DISTANCE_FROM_FINISH;
}

/****************************************/
/****************************************/

void SingleRobotLoopFunction::Reset()
{
   this->stepCount = 0;
   this->totalStepProximity = 0;
   this->totalStepRightWheelSpeed = 0;
   this->totalStepLeftWheelSpeed = 0;
   this->totalOnSegmentPerformance = 0;

   const CVector3 startingPosition = GetRandomStartingPosition();
   /*
    * Move robot to the initial position 
    */
   if (!MoveEntity(
           footBot->GetEmbodiedEntity(),
           startingPosition,
           CQuaternion().FromEulerAngles(CRadians::ZERO, CRadians::ZERO, CRadians::ZERO),
           false))
   {
      LOGERR << "Can't move robot in <" << startingPosition << ">" << std::endl;
   }
}

/****************************************/
/****************************************/

void SingleRobotLoopFunction::ConfigureFromGenome(const GARealGenome &c_genome)
{
   /* Copy the genes into the NN parameter buffer */
   for (size_t i = 0; i < controller->GENOME_SIZE; ++i)
   {
      parameters[i] = c_genome[i];
   }
   /* Set the NN parameters */
   controller->GetPerceptron().SetOnlineParameters(controller->GENOME_SIZE, parameters);
}

/****************************************/
/****************************************/

Real SingleRobotLoopFunction::Performance()
{
   size_t segmentsAnalyzed = this->stepCount / SEGMENT_LENGTH;
   const Real avgOnSegmentPerformance = totalOnSegmentPerformance / segmentsAnalyzed;
   const Real dist = DistanceFromSegment(GetFootBotPosition(), this->finishSegmentV1, this->finishSegmentV2);
   const Real reachFinishLine = 1 / (1 + dist);
   return avgOnSegmentPerformance * reachFinishLine / this->stepCount;
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(SingleRobotLoopFunction, "single_robot_loop_function");
