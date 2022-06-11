#include "problemdescription.h"
#include "ga.h"

int main(){
	ProblemDescription problemDescription("Data/DataFiles/p01");

	Parameters parameters;
	parameters.populationSize = 1000;
    parameters.crossoverRate = 0.8;
    parameters.intraDepotMutationRate = 0.2;
    parameters.interDepotMutationRate = 0.25;
    parameters.interDepotMutationAttemptRate = 10;

	int maxGenerations = 20000;

    optimize(maxGenerations, parameters, problemDescription);
    return 0;
}
