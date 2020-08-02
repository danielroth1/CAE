#include "PlasticityDemo.h"

#include <ApplicationControl.h>

#include <simulation/rigid/RigidBody.h>

#include <simulation/constraints/BallJoint.h>

#include <simulation/forces/LinearForce.h>

#include <scene/data/geometric/GeometricDataFactory.h>
#include <scene/data/geometric/GeometricPoint.h>

#include <modules/mesh_converter/MeshCriteria.h>

using namespace Eigen;

PlasticityDemo::PlasticityDemo(ApplicationControl* ac)
    : mAc(ac)
{

}

std::string PlasticityDemo::getName()
{
    return "Plasticity";
}

void PlasticityDemo::load()
{
    SGControl* sg = mAc->getSGControl();

    double boxDim = 1.0;
    MeshCriteria criteria(0.0, 0.0, 0.0, 0.2, 0.0, false, 0.0);
    SGLeafNode* plasticNode = sg->createLeafNode(
                "Plastic Box", sg->getSceneGraph()->getRoot(),
                std::make_shared<Polygon3D>(
                    GeometricDataFactory::create3DBox(boxDim, boxDim, boxDim, criteria)),
                Eigen::Vector3d::Zero(), true);

    sg->createFEMObject(plasticNode->getData());

    ElasticMaterial material;
    material.setFromYoungsPoisson(1000, 0.45);
    material.setPlasticYield(0.3);
    material.setPlasticCreep(2.5);
    material.setPlasticMaxStrain(100.0);

    std::shared_ptr<FEMObject> femObj =
            std::static_pointer_cast<FEMObject>(plasticNode->getData()->getSimulationObject());
    femObj->setElasticMaterial(material);

    std::vector<ID> truncatedIds = { 0 };
    femObj->addTrunctionIds(truncatedIds);
}

void PlasticityDemo::unload()
{
}
