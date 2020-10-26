/* GA-related headers */
#include <ga/ga.h>
#include <math.h>
#include <time.h>

/* ARGoS-related headers */
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/loop_functions.h>

#include <loop_function/single_robot_loop_function.h>

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
    static SingleRobotLoopFunction &cLoopFunctions =
        dynamic_cast<SingleRobotLoopFunction &>(cSimulator.GetLoopFunctions());
    /*
    * Run multiple trials and take the worst performance as final value.
    */
    Real worstPerformance = 1000;

    srand(time(NULL));
    for (size_t i = 0; i < NUM_TRIALS; ++i)
    {
        cSimulator.SetRandomSeed(rand());
        /* Reset the experiment.*/
        cSimulator.Reset();
        /* Configure the controller with the genome */
        cLoopFunctions.ConfigureFromGenome(cRealGenome);
        /* Run the experiment */
        cSimulator.Execute();
        /* Update performance */
        worstPerformance = Min(worstPerformance, cLoopFunctions.Performance());
    }

    return worstPerformance;
}


/*
 * Flush best individual
 */
int FlushBest(const GARealGenome &c_genome,
              size_t un_generation,
              char *folderName)
{
    std::ostringstream cOSS;
    cOSS << folderName << "/"
         << "best_" << un_generation << ".dat";
    std::ofstream cOFS(cOSS.str().c_str(), std::ios::out | std::ios::trunc);
    if (cOFS.is_open())
    {
        cOFS << GeneticController::GENOME_SIZE // first write the number of values to dump
             << " "
             << c_genome // then write the actual values
             << "\n";
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
    if (argc < 2)
    {
        std::cerr << "Pass the folder name where the algorithm intermediate files will be stored." << std::endl;
        return 1;
    }
    char *folder = argv[1];
    /*
    * Initialize GALIB
    */
    GAAlleleSet<float> cAlleleSet(-15.0f, 15.0f);
    /* Create a genome using LaunchARGoS() to evaluate it */
    GARealGenome cGenome(GeneticController::GENOME_SIZE, cAlleleSet, LaunchARGoS);

    GASteadyStateGA cGA(cGenome);
    cGA.maximize(); // the objective function must be maximized
    cGA.crossover(GARealOnePointCrossover);

    // load parameters
    std::ifstream parametersFile("genetic_parameters.conf");
    cGA.parameters(parametersFile, GABoolean::gaTrue);
    LOG << "Algorithm parameters: \n"
        << cGA.parameters() << std::endl;

    //write algorith marameters in the a conf file inside the algorithm sfolder
    std::ostringstream cOSS;
    cOSS << folder << "/"
         << "parameters.conf";
    std::ofstream cOFS(cOSS.str().c_str(), std::ios::out | std::ios::trunc);
    cOFS << cGA.parameters()
         << "\n";
    cOFS.close();

    /*
    * Initialize ARGoS
    */
    /* The CSimulator class of ARGoS is a singleton. Therefore, to
    * manipulate an ARGoS experiment, it is enough to get its instance */
    argos::CSimulator &cSimulator = argos::CSimulator::GetInstance();
    /* Set the .argos configuration file */
    cSimulator.SetExperimentFileName("train.argos");
    /* Load it to configure ARGoS */
    cSimulator.LoadExperiment();

    /*
    * Launch the evolution, setting the random seed
    */
    cGA.initialize(132);
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
            if (FlushBest(
                    dynamic_cast<const GARealGenome &>(cGA.statistics().bestIndividual()),
                    cGA.generation(),
                    folder) == 0)
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
