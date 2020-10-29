/* GA-related headers */
#include <ga/ga.h>
#include <math.h>
#include <time.h>

/* ARGoS-related headers */
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/loop_functions.h>

#include <loop_function/race_loop_function.h>

using namespace std;
/****************************************/
/****************************************/

vector<string> LaunchARGoS(int numRaces)
{
    static argos::CSimulator &cSimulator = argos::CSimulator::GetInstance();
    static RaceLoopFunction &loopFunction = dynamic_cast<RaceLoopFunction &>(cSimulator.GetLoopFunctions());

    vector<string> winners;
     ostringstream cOSS;
    cOSS << "collisions.txt";
    ofstream cOFS(cOSS.str().c_str(), ios::out | ios::trunc);
    /*
    * Run multiple trials and save winners
    */
    for (size_t i = 0; i < numRaces; i++)
    {
        cSimulator.SetRandomSeed(rand());
        /* Reset the experiment.*/
        cSimulator.Reset();
        /* Run the experiment */
        cSimulator.Execute();
        /* Update performance */
        winners.push_back(loopFunction.Winner());
        loopFunction.PrintNumCollisions(cOFS);
    }
    return winners;
}

#define DEFAULT_NUM_RACES 10
int main(int argc, char **argv)
{
    srand(time(NULL));

    int numRaces = DEFAULT_NUM_RACES;
    if (argc >= 2)
        numRaces = atoi(argv[1]);
    /*
    * Initialize ARGoS
    */
    /* The CSimulator class of ARGoS is a singleton. Therefore, to
    * manipulate an ARGoS experiment, it is enough to get its instance */
    argos::CSimulator &cSimulator = argos::CSimulator::GetInstance();
    /* Set the .argos configuration file */
    cSimulator.SetExperimentFileName("race.argos");

    /* Load it to configure ARGoS */
    cSimulator.LoadExperiment();

    auto winners = LaunchARGoS(numRaces);
    ostringstream cOSS;
    cOSS << "race_winners.txt";
    ofstream cOFS(cOSS.str().c_str(), ios::out | ios::trunc);
    for (size_t i = 0; i < winners.size(); i++)
    {
        cOFS << i << "\t" << winners[i] << endl;
    }
    cOFS.close();

    cSimulator.Destroy();

    /* All is OK */
    return 0;
}

/****************************************/
/****************************************/
