#include "SimulationPointRefVector.h"

SimulationPointRefVector::SimulationPointRefVector(SimulationObject* object)
    : PointRefCollectionVector<SimulationObject*, Eigen::Vector&>(object)
{

}

Eigen::Vector& SimulationPointRefVector::getPoint(ID index)
{
    return mObject->getPosition(index);
}
