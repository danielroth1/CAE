#include "SimulationPointRefMap.h"

SimulationPointRefMap::SimulationPointRefMap()
    : PointRefCollectionMap<SimulationObject*, Eigen::Vector&>()
{

}

Eigen::Vector& SimulationPointRefMap::getPoint(SimulationObject* object, ID index)
{
    return object->getPosition(mMap[object][index]);
}
