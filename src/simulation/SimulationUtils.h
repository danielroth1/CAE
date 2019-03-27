#ifndef SIMULATIONUTILS_H
#define SIMULATIONUTILS_H

#include "data_structures/DataStructures.h"

class SimulationUtils
{
public:

    // std::vector operations
    static void setVectorZero(Vectors& vs);
    static void resizeVectorWithZeros(Vectors& vs, unsigned int size);

private:
    SimulationUtils();
};

#endif // SIMULATIONUTILS_H
