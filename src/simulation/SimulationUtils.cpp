#include "SimulationUtils.h"

using namespace Eigen;

void SimulationUtils::setVectorZero(Vectors& vs)
{
    Index size = static_cast<Index>(vs.size());
    VectorXd::Map(vs[0].data(), size * 3) = VectorXd::Zero(size * 3);
}

void SimulationUtils::resizeVectorWithZeros(Vectors& vs, unsigned int size)
{
    vs.resize(size);
    setVectorZero(vs);
}

SimulationUtils::SimulationUtils()
{

}
