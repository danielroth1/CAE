#include "ConstrainedDeformableDemo.h"

#include <ApplicationControl.h>

#include <modules/mesh_converter/MeshCriteria.h>

#include <scene/model/RenderModel.h>

#include <simulation/constraints/BallJoint.h>

#include <scene/data/geometric/Polygon3D.h>

ConstrainedDeformableDemo::ConstrainedDeformableDemo(ApplicationControl* ac)
    : mAc(ac)
{

}

std::string ConstrainedDeformableDemo::getName()
{
    return "Constrained Deformable";
}

void ConstrainedDeformableDemo::load()
{
    SGControl* sg = mAc->getSGControl();
    SimulationControl* sc = mAc->getSimulationControl();

    SGLeafNode* floorNode =
            sg->createBox(
                "Floor",
                sg->getSceneGraph()->getRoot(),
                Vector(0.0, -0.25, 0.0), 12, 0.5, 12, true);

    sg->createRigidBody(floorNode->getData(), 1.0, true);
    sg->createCollidable(floorNode->getData());

    SGLeafNode* wallNode =
            sg->createBox(
                "Wall",
                sg->getSceneGraph()->getRoot(),
                Vector(-4.25, 3, 0), 0.5, 4, 6, true);
    sg->createRigidBody(wallNode->getData(), 1.0, false);
    sg->createCollidable(wallNode->getData());

    SGLeafNode* beamNode =
            sg->createBox(
                "Beam",
                sg->getSceneGraph()->getRoot(),
                Vector(-2.0, 2, 0), 3, 1, 1, true);
    MeshCriteria criteria(0.0, 0.0, 0.0, 0.0, 0.0, true, 60.0);
    sg->create3DGeometryFrom2D(beamNode, criteria, true);
    sg->createFEMObject(beamNode->getData(), 1.0);
    sg->createCollidable(beamNode->getData());
    beamNode->getData()->getRenderModel()->setWireframeEnabled(true);

    // fix the 12 leftmost vertices of the beam

    // first find them...
    std::vector<ID> leftMostVertexIds;
    std::vector<std::tuple<double, ID>> xCoordVertexIds;
    Polygon3D* poly = static_cast<Polygon3D*>(beamNode->getData()->getGeometricDataRaw());
    for (size_t i = 0; i < poly->getPositions().size(); ++i)
    {
        xCoordVertexIds.push_back(std::make_tuple(poly->getPositions()[i][0], i));
    }
    std::sort(xCoordVertexIds.begin(), xCoordVertexIds.end(),
              [](const std::tuple<double, ID>& t1,
              const std::tuple<double, ID>& t2)
    {
        return std::get<0>(t1) < std::get<0>(t2);
    });
    for (size_t i = 0; i < 12; ++i)
    {
        leftMostVertexIds.push_back(std::get<1>(xCoordVertexIds[i]));
    }

    // ...then fix them to the wall
    for (size_t i = 0; i < leftMostVertexIds.size(); ++i)
    {
        ID vertexId = leftMostVertexIds[i];
        AbstractPolygon* wallPoly = static_cast<AbstractPolygon*>(wallNode->getData()->getGeometricDataRaw());
        Eigen::Vector wallCOM = wallPoly->getTransform().translation();
        Eigen::Vector wallR = poly->getPosition(vertexId) - wallCOM;

        std::shared_ptr<BallJoint> joint = std::make_shared<BallJoint>(
                    SimulationPointRef(beamNode->getData()->getSimulationObjectRaw(),
                                       vertexId),
                    SimulationPointRef(wallNode->getData()->getSimulationObjectRaw(),
                                       static_cast<AbstractPolygon*>(wallNode->getData()->getGeometricDataRaw()),
                                       wallR));

        sc->addConstraint(joint);
    }
}

void ConstrainedDeformableDemo::unload()
{

}
