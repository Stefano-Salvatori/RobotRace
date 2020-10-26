#ifndef GENETIC_LOOP_FUNCTION_H
#define GENETIC_LOOP_FUNCTION_H

/* Utility */
#include <utility>
#include <utils/utils.h>

/* The NN controller */
#include <controllers/genetic_controller/genetic_controller.h>
#include <controllers/motor_schema_controller/motor_schema.h>

/* ARGoS-related headers */
#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/general.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
#include <argos3/plugins/simulator/entities/box_entity.h>

/* GA-related headers */
#include <ga/ga.h>
#include <ga/GARealGenome.h>
#include <ga/GARealGenome.C> // this is necessary!

#define NUM_OBSTACLES 6
#define DEFAULT_OBSTACLE_MIN_SIZE 0.1
#define DEFAULT_OBSTACLE_MAX_SIZE 0.8

using namespace argos;

class SingleRobotLoopFunction : public CLoopFunctions
{
private:
   size_t stepCount;
   Real totalStepProximity;
   Real totalStepRightWheelSpeed;
   Real totalStepLeftWheelSpeed;

   Real totalOnSegmentPerformance;

   //Finish line of the race (as a pair of point that defines the vertices of a segment)
   CVector2 finishSegmentV1;
   CVector2 finishSegmentV2;

   //Starting line of the race (as a pair of point that defines the vertices of a segment)
   CVector2 startingSegmentV1;
   CVector2 startingSegmentV2;
   CFootBotEntity *footBot;
   MotorSchemaController *controller;
   Real *parameters;
   CRandom::CRNG *m_pcRNG;

   CBoxEntity *obstacles[NUM_OBSTACLES];
   Real obstaclesMinSize = DEFAULT_OBSTACLE_MIN_SIZE;
   Real obstaclesMaxSize = DEFAULT_OBSTACLE_MAX_SIZE;

   inline CVector2 GetFootBotPosition()
   {
      CVector2 cPos;
      cPos.Set(footBot->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
               footBot->GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
      return cPos;
   }

   inline CVector3 GetRandomStartingPosition()
   {
      CRange<Real> startingXRange = RangeX(this->startingSegmentV1, this->startingSegmentV2);
      CRange<Real> startingYRange = RangeY(this->startingSegmentV1, this->startingSegmentV2);
      /* Set the position uniformly in the starting line*/
      const Real startingX = m_pcRNG->Uniform(startingXRange);
      const Real startingY = m_pcRNG->Uniform(startingYRange);
      return CVector3(startingX, startingY, Real(0));
   }

   void AddObstacles();
   void RemoveObstacles();

public:
   SingleRobotLoopFunction();
   virtual ~SingleRobotLoopFunction();

   virtual void Init(TConfigurationNode &t_node);
   virtual void PreStep();
   virtual void PostStep();
   virtual void Reset();
   virtual bool IsExperimentFinished();

   /* Configures the robot controller from the genome */
   void ConfigureFromGenome(const GARealGenome &c_genome);

   /* Calculates the performance of the robot in a trial */
   Real Performance();
};

#endif
