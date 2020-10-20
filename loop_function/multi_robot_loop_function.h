#ifndef MULTIPLE_ROBOT_LOOP_FUNCTION_H
#define MULTIPLE_ROBOT_LOOP_FUNCTION_H

/* Utility */
#include <utility>
#include <utils/utils.h>

/* The NN controller */
#include <controllers/advanced_controller/advanced_controller.h>

/* ARGoS-related headers */
#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/general.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>

/* GA-related headers */
#include <ga/ga.h>
#include <ga/GARealGenome.h>
#include <ga/GARealGenome.C> // this is necessary!

/****************************************/
/****************************************/


/****************************************/
/****************************************/

using namespace argos;

class MultiRobotLoopFunction : public CLoopFunctions
{
private:
    size_t currentGeneration;
    size_t stepCount;
    Real totalStepProximity;
    Real totalStepRightWheelSpeed;
    Real totalStepLeftWheelSpeed;

    Real stepByStepPerformance;

    //Finish line of the race (as a pair of point that defines the vertices of a segment)
    CVector2 finishSegmentV1;
    CVector2 finishSegmentV2;

    //Starting line of the race (as a pair of point that defines the vertices of a segment)
    CVector2 startingSegmentV1;
    CVector2 startingSegmentV2;

    CFootBotEntity *mainFootbot;
    AdvancedController *mainController;


    CFootBotEntity *opponentFootbot;
    AdvancedController *opponentController;

    Real *controllerParameters;
    CRandom::CRNG *m_pcRNG;

    inline CVector2 GetFootbotPosition(CFootBotEntity *footbot)
    {
        CVector2 cPos;
        cPos.Set(footbot->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                 footbot->GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
        return cPos;
    }

public:
    MultiRobotLoopFunction();
    virtual ~MultiRobotLoopFunction();

    virtual void Init(TConfigurationNode &t_node);
    virtual void PreStep();
    virtual void PostStep();
    virtual void Reset();
    virtual bool IsExperimentFinished();

    inline void SetCurrentGeneration(size_t gen) {
        currentGeneration = gen;
    }

    /* Configures the controllers from the genome */
    void ConfigureFromGenome(const GARealGenome &c_genome);

    /* Calculates the performance of the robot in a trial */
    Real Performance();
};

#endif
