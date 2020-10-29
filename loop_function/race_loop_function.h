#ifndef RACE_LOOP_FUNCTION_H
#define RACE_LOOP_FUNCTION_H

/* Utility */
#include <utility>
#include "utils/utils.h"

/* The NN controller */
#include <controllers/genetic_controller/genetic_controller.h>

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

#define DEFAULT_NUM_BOTS 2
#define NUM_OBSTACLES 6
#define DEFAULT_OBSTACLE_MIN_SIZE 0.1
#define DEFAULT_OBSTACLE_MAX_SIZE 0.8

using namespace argos;

class RaceLoopFunction : public CLoopFunctions
{
private:
   size_t stepCount = 0;
   int numBots = DEFAULT_NUM_BOTS;
   //Finish line of the race (as a pair of point that defines the vertices of a segment)
   CVector2 finishSegmentV1;
   CVector2 finishSegmentV2;

   //Starting line of the race (as a pair of point that defines the vertices of a segment)
   CVector2 startingSegmentV1;
   CVector2 startingSegmentV2;

   std::vector<CFootBotEntity> bots;

   CRandom::CRNG *m_pcRNG;

   CBoxEntity *obstacles[NUM_OBSTACLES];
   Real obstaclesMinSize = DEFAULT_OBSTACLE_MIN_SIZE;
   Real obstaclesMaxSize = DEFAULT_OBSTACLE_MAX_SIZE;

   inline CVector2 GetFootBotPosition(size_t index = 0)
   {
      CVector2 cPos;
      cPos.Set(bots[index].GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
               bots[index].GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
      return cPos;
   }

   void AddObstacles();
   void RemoveObstacles();

public:
   RaceLoopFunction();
   virtual ~RaceLoopFunction();

   virtual void Init(TConfigurationNode &t_node);
   virtual void PreStep();
   virtual void PostStep();
   virtual void Reset();
   virtual bool IsExperimentFinished();

   /**
    * Return the index of the winning robot
    */
   std::string Winner();

   inline void PrintNumCollisions(std::ostream& output)
   {
      for (size_t i = 0; i < bots.size(); i++)
      {
         size_t n = (&dynamic_cast<CommonController &>(bots[i].GetControllableEntity().GetController()))->GetNumCollisions();
         output << bots[i].GetControllableEntity().GetController().GetId() << "\t" << n << std::endl;
      }
   }
};

#endif
