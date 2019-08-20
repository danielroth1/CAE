#include "SceneLeafData.h"
#include <scene/data/GeometricData.h>
//#include <scene/data/SimulationData.h>
#include <scene/model/RenderModel.h>
#include <simulation/SimulationObject.h>


SceneLeafData::SceneLeafData(
        Node<std::shared_ptr<SceneData>, std::shared_ptr<SceneLeafData>>* node)
    : SceneData (node)
{
    mGeometricData = nullptr;
    mSimulationObject = nullptr;
    mRenderModel = nullptr;
}

SceneLeafData::~SceneLeafData()
{

}

void SceneLeafData::accept(SceneDataVisitor* visitor)
{
    visitor->visit(this);
}

bool SceneLeafData::isLeafData()
{
    return true;
}

void SceneLeafData::setVisible(bool visible)
{
    if (mRenderModel)
        mRenderModel->setVisible(visible);
}

void SceneLeafData::setGeometricData(std::shared_ptr<GeometricData> geometricData)
{
    mGeometricData = geometricData;
}

void SceneLeafData::setSimulationObject(std::shared_ptr<SimulationObject> simulationObject)
{
    mSimulationObject = simulationObject;
}

void SceneLeafData::setRenderModel(std::shared_ptr<RenderModel> renderModel)
{
    mRenderModel = renderModel;
}

//Vector& SceneLeafData::getPosition(ID vertexID)
//{
//    return mGeometricData->getPosition(vertexID);
//}

bool SceneLeafData::isVisible()
{
    if (mRenderModel)
        return mRenderModel->isVisible();
    else
        return false;
}

GeometricData* SceneLeafData::getGeometricDataRaw()
{
    return mGeometricData.get();
}

SimulationObject* SceneLeafData::getSimulationObjectRaw()
{
    return mSimulationObject.get();
}

RenderModel* SceneLeafData::getRenderModelRaw()
{
    return mRenderModel.get();
}

std::shared_ptr<GeometricData> SceneLeafData::getGeometricData()
{
    return mGeometricData;
}

std::shared_ptr<SimulationObject> SceneLeafData::getSimulationObject()
{
    return mSimulationObject;
}

std::shared_ptr<RenderModel> SceneLeafData::getRenderModel()
{
    return mRenderModel;
}
