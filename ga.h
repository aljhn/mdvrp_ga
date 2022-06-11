#ifndef GA_H
#define GA_H

#include "problemdescription.h"

typedef struct {
    int populationSize;
    double crossoverRate;
    double intraDepotMutationRate;
    double interDepotMutationRate;
    int interDepotMutationAttemptRate;
} Parameters;

void optimize(int maxIterations, Parameters &parameters, ProblemDescription &problemDescription);

#endif
