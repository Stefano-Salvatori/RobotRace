/* GA-related headers */
#include <ga/ga.h>
#include <math.h>
#include <time.h>

/* ARGoS-related headers */
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/loop_functions.h>

#include <loop_function/single_robot_loop_function.h>

/*
 * Launch ARGoS to evaluate a genome.
 */
void LaunchARGoS(int numTrials, double results[])
{
    static argos::CSimulator &cSimulator = argos::CSimulator::GetInstance();
    /* Get a reference to the loop functions */
    static SingleRobotLoopFunction &cLoopFunctions = dynamic_cast<SingleRobotLoopFunction &>(cSimulator.GetLoopFunctions());
    /*
    * Run multiple trials and save results
    */
    srand(time(NULL));
    for (size_t i = 0; i < numTrials; ++i)
    {
        cSimulator.SetRandomSeed(rand());
        /* Reset the experiment.*/
        cSimulator.Reset();
        /* Run the experiment */
        cSimulator.Execute();
        /* Update performance */
        results[i] = cLoopFunctions.Performance();
        // tot +=cLoopFunctions.Performance();
    }
}

/****************************************/
/****************************************/

#define DEFAULT_NUM_TRIALS 10
int main(int argc, char **argv)
{
    int numTrials = DEFAULT_NUM_TRIALS;
    if (argc >= 2)
        numTrials = std::atoi(argv[1]);
   
    argos::CSimulator &cSimulator = argos::CSimulator::GetInstance();
    cSimulator.SetExperimentFileName("test.argos");
    cSimulator.LoadExperiment();

    double results[numTrials];
    LaunchARGoS(numTrials, results);
    std::ostringstream cOSS;
    cOSS << "results.txt";
    std::ofstream cOFS(cOSS.str().c_str(), std::ios::out | std::ios::trunc);
    for (size_t i = 0; i < numTrials; i++)
    {
        cOFS << i << "\t" << results[i] << "\n";
    }
    cOFS.close();

    /*
    * Dispose of ARGoS stuff
    */
    cSimulator.Destroy();

    /* All is OK */
    return 0;
}
