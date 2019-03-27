#include "UniqueVertex.h"


#include "simulation/SimulationObject.h"

UniqueVertex::UniqueVertex()
    : mSceneLeafData(nullptr)
    , mVectorID(-1)
{

}

UniqueVertex::UniqueVertex(SceneLeafData* sceneLeafData, ID vectorID)
    : mSceneLeafData(sceneLeafData)
    , mVectorID(vectorID)
{

}

//bool UniqueVertex::operator<(const UniqueVertex &v) const
//{
//    if (mSimulationObjectId < v.getSimulationObjectId())
//        return true;
//    if (mSimulationObjectId == v.getSimulationObjectId() &&
//            mVertexId < v.getVertexId())
//        return true;
//    return false;
//}

SceneLeafData* UniqueVertex::getSceneLeafData() const
{
    return mSceneLeafData;
}

ID UniqueVertex::getVectorID() const
{
    return mVectorID;
}

Eigen::Vector& UniqueVertex::getVector() const
{
    // TODO: replace with simulation/ geometric reference
    return mSceneLeafData->getSimulationObjectRaw()->getPosition(mVectorID);
}
