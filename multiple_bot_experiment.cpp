/* GA-related headers */
#include <ga/ga.h>

/* ARGoS-related headers */
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/loop_functions.h>

#include <loop_function/multi_robot_loop_function.h>

//Number of trials foreach genome
#define NUM_TRIALS 5

/****************************************/
/****************************************/

/*
 * Launch ARGoS to evaluate a genome.
 */
float LaunchARGoS(GAGenome &c_genome)
{
    /* Convert the received genome to the actual genome type */
    GARealGenome &cRealGenome = dynamic_cast<GARealGenome &>(c_genome);
    /* The CSimulator class of ARGoS is a singleton. Therefore, to
    * manipulate an ARGoS experiment, it is enough to get its instance.
    * This variable is declared 'static' so it is created
    * once and then reused at each call of this function.
    * This line would work also without 'static', but written this way
    * it is faster. */
    static argos::CSimulator &cSimulator = argos::CSimulator::GetInstance();
    /* Get a reference to the loop functions */
    static MultiRobotLoopFunction &cLoopFunctions =
        dynamic_cast<MultiRobotLoopFunction &>(cSimulator.GetLoopFunctions());
    cLoopFunctions.SetCurrentGeneration(c_genome.geneticAlgorithm()->generation());

    /*
    * Run multiple trials and take the worst performance as final value.
    */
    Real worstPerformance = 100000;
    for (size_t i = 0; i < NUM_TRIALS; ++i)
    {
        /* Reset the experiment.*/
        cSimulator.Reset();
        /* Configure the controller with the genome */
        cLoopFunctions.ConfigureFromGenome(cRealGenome);
        /* Run the experiment */
        cSimulator.Execute();
        /* Update performance */
        worstPerformance = Min(worstPerformance, cLoopFunctions.Performance());
    }
    /* Return the result of the evaluation */
    return worstPerformance;
}

/*
 * Flush best individual
 */
int FlushBest(const GARealGenome &c_genome,
              size_t un_generation)
{
    std::ostringstream cOSS;
    cOSS << "best_multiple/"
         << "best_" << un_generation << ".dat";
    std::ofstream cOFS(cOSS.str().c_str(), std::ios::out | std::ios::trunc);
    if (cOFS.is_open())
    {
        cOFS << AdvancedController::GENOME_SIZE // first write the number of values to dump
             << " "
             << c_genome // then write the actual values
             << std::endl;
        cOFS.close();
        return 0;
    }
    else
    {
        return 1;
    }
}

/****************************************/
/****************************************/

int main(int argc, char **argv)
{
    /*
    * Initialize GALIB
    */
    /* Create an allele whose values can be in the range [-20,20] */
    GAAlleleSet<float> cAlleleSet(-20.0f, 20.0f);
    /* Create a genome using LaunchARGoS() to evaluate it */
    GARealGenome cGenome(AdvancedController::GENOME_SIZE, cAlleleSet, LaunchARGoS);
    GASteadyStateGA cGA(cGenome);
    cGA.maximize(); // the objective function must be maximized

    // load parameters
    std::ifstream parametersFile("genetic_parameters.conf");
    cGA.parameters(parametersFile, GABoolean::gaTrue);
    LOG << "Algorithm parameters: \n"
        << cGA.parameters() << std::endl;

    cGA.crossover(GARealOnePointCrossover);

    // foreach generation save best and avg score
    cGA.selectScores(GAStatistics::AllScores);

    /*
    * Initialize ARGoS
    */
    /* The CSimulator class of ARGoS is a singleton. Therefore, to
    * manipulate an ARGoS experiment, it is enough to get its instance */
    argos::CSimulator &cSimulator = argos::CSimulator::GetInstance();
    /* Set the .argos configuration file
    * This is a relative path which assumed that you launch the executable
    * from argos3-examples (as said also in the README) */
    cSimulator.SetExperimentFileName("multibot_train.argos");
    /* Load it to configure ARGoS */
    cSimulator.LoadExperiment();

    /*
    * Launch the evolution, setting the random seed
    */
    cGA.initialize(12345);
    do
    {
        argos::LOG << "Generation #" << cGA.generation() << "...";
        cGA.step();
        argos::LOG << "done.";
        if (cGA.generation() % cGA.flushFrequency() == 0)
        {
            /* Flush scores */
            argos::LOG << "   Flushing...";
            cGA.flushScores();
            argos::LOG << "done.";

            /* Flush best individual */
            argos::LOG << "   Flushing best individual: "
                       << cGA.population().max()
                       << " population avg: "
                       << cGA.population().ave()
                       << "...";
            if (FlushBest(dynamic_cast<const GARealGenome &>(cGA.statistics().bestIndividual()), cGA.generation()) == 0)
            {
                argos::LOG << " done.";
            }
            else
            {
                argos::LOG << "Unable to open file to save best_" << cGA.generation();
            }
        }
        LOG << std::endl;
        LOG.Flush();
    } while (!cGA.done());

    /*
    * Dispose of ARGoS stuff
    */
    cSimulator.Destroy();

    /* All is OK */
    return 0;
}

/****************************************/
/****************************************/
