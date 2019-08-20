#pragma once

#ifndef SCENELEAFDATA_H
#define SCENELEAFDATA_H

// Includes
#include "SceneData.h"
#include "SceneDataVisitor.h"

#include <data_structures/DataStructures.h>


// Forward Declarations
class GeometricData;
//class SimulationData;
class SimulationObject;
class RenderModel;


class SceneLeafData : public SceneData
{
public:
    SceneLeafData(
            Node<std::shared_ptr<SceneData>, std::shared_ptr<SceneLeafData>>* node);

    virtual ~SceneLeafData() override;

    // SceneData methods
public:
    virtual void accept(SceneDataVisitor* visitor) override;

    virtual bool isLeafData() override;

    // Setters
    // Sets the visibility of the RenderModel. If there is none, does nothing.
    void setVisible(bool visible);
    void setGeometricData(std::shared_ptr<GeometricData> geometricData);
    void setSimulationObject(std::shared_ptr<SimulationObject> simulationObject);
    void setRenderModel(std::shared_ptr<RenderModel> renderModel);

    // Getters
//    Eigen::Vector& getPosition(ID vertexID);

    // Returns the visibility of the RenderModel. If there is none, returns
    // false.
    virtual bool isVisible();

    // Returns raw pointer, use this for non-ownership access
    GeometricData* getGeometricDataRaw();

    // Returns raw pointer, use this for non-ownership access
    SimulationObject* getSimulationObjectRaw();

    // Returns raw pointer, use this for non-ownership access
    RenderModel* getRenderModelRaw();

    // getter to pass ownership
    std::shared_ptr<GeometricData> getGeometricData();

    // getter to pass ownership
    std::shared_ptr<SimulationObject> getSimulationObject();

    // getter to pass ownership
    std::shared_ptr<RenderModel> getRenderModel();

private:
    // members of the data that a.o. is used to initialize renderer/ simulation
    // TODO: use smart pointers here
    std::shared_ptr<GeometricData> mGeometricData;
    std::shared_ptr<SimulationObject> mSimulationObject;
    std::shared_ptr<RenderModel> mRenderModel;

    // Constraints?
};

#endif // SCENELEAFDATA_H
