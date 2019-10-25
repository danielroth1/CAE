#include "CubeWallDemo.h"

#include <scene/scene_graph/SGControl.h>
#include <scene/scene_graph/SGCore.h>

#include <modules/mesh_converter/MeshCriteria.h>

#include <ApplicationControl.h>
#include <SimulationControl.h>

#include <simulation/fem/FEMObject.h>

using namespace Eigen;

CubeWallDemo::CubeWallDemo(ApplicationControl* ac,
                           bool rigid)
    : mAc(ac)
    , mRigid(rigid)
{
    if (rigid)
        mName = "Cube Wall (Rigid)";
    else
        mName = "Cube Wall (Deformable)";
}

std::string CubeWallDemo::getName()
{
    return mName;
}

void CubeWallDemo::load()
{
    // Floor
    SGLeafNode* node2 = mAc->getSGControl()->createBox("Floor", mAc->getSGControl()->getSceneGraph()->getRoot(),
                                              /*Vector(-5, -2, -5)*/Vector(0.0, -1.5, 0.0), 12, 0.5, 12, true);

    mAc->getSGControl()->createRigidBody(node2->getData(), 1.0, true);
    mAc->getSGControl()->createCollidable(node2->getData());

    bool createSingleBox = false;
    if (createSingleBox)
    {
        mAc->getSimulationControl()->setGravity(Eigen::Vector::Zero());

        MeshCriteria criteria(0.0, 0.0, 0.0, 0.2, 1.0, true);
        SGLeafNode* node1 = mAc->getSGControl()->createBox(
                    "Box", mAc->getSGControl()->getSceneGraph()->getRoot(),
                    Vector(-1, -0.5, 0.0), 1.5, 1.0, 0.5, true);
        mAc->getSGControl()->create3DGeometryFrom2D(node1, criteria);
        mAc->getSGControl()->createFEMObject(node1->getData());
        mAc->getSGControl()->createCollidable(node1->getData());
    }
    else
    {
        // some boxes:
        for (int r = 0; r < 6; ++r)
        {
            for (int c = 0; c < 3; ++c)
            {
                for (int z = 0; z < 3; ++z)
                {

                    if (!mRigid)
                    {
                        MeshCriteria criteria(0.0, 0.0, 0.0, 0.0, 0.0, true, 0.0);

                        SGLeafNode* node1 = mAc->getSGControl()->createBox(
                                    "Box", mAc->getSGControl()->getSceneGraph()->getRoot(),
                                    Vector(-1 + 0.6 * c, -0.5 + 0.7 * r, -1 + 0.6 * z),
                                    0.5, 0.5, 0.5, true);
                        mAc->getSGControl()->create3DGeometryFrom2D(node1, criteria);
                        mAc->getSGControl()->createFEMObject(node1->getData());
                        mAc->getSGControl()->createCollidable(node1->getData());

                        std::shared_ptr<FEMObject> femObj =
                                std::dynamic_pointer_cast<FEMObject>(
                                    node1->getData()->getSimulationObject());
                        femObj->setYoungsModulus(5e+4);
                    }
                    else
                    {
                        SGLeafNode* node1 = mAc->getSGControl()->createBox(
                                    "Box", mAc->getSGControl()->getSceneGraph()->getRoot(),
                                    Vector(-1 + 0.6 * c, -0.5 + 0.7 * r, -1 + 0.6 * z),
                                    0.5, 0.5, 0.5, true);
                        mAc->getSGControl()->createRigidBody(node1->getData(), 1.0, false);
                        mAc->getSGControl()->createCollidable(node1->getData());
                    }
                }

            }
        }
    }
}

void CubeWallDemo::unload()
{

}
