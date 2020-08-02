#include "CarCrashDemo.h"

#include <ApplicationControl.h>
#include <QCoreApplication>

#include <simulation/rigid/RigidBody.h>

#include <simulation/constraints/BallJoint.h>

#include <simulation/forces/LinearForce.h>
#include <simulation/forces/RotationalMotor.h>

#include <scene/data/geometric/GeometricDataFactory.h>
#include <scene/data/geometric/GeometricDataFactory.h>
#include <scene/data/geometric/GeometricPoint.h>
#include <scene/data/geometric/MeshInterpolatorFEM.h>
#include <scene/data/geometric/Polygon2D.h>

#include <modules/mesh_converter/MeshCriteria.h>

#include <scene/model/PolygonRenderModel.h>
#include <scene/model/RenderModel.h>

#include <rendering/Appearance.h>
#include <rendering/Appearances.h>

#include <modules/interpolator/InterpolatorModule.h>

using namespace Eigen;

CarCrashDemo::CarCrashDemo(ApplicationControl* ac)
    : mAc(ac)
{

}

std::string CarCrashDemo::getName()
{
    return "Car Crash";
}

void CarCrashDemo::load()
{
    SGControl* sg = mAc->getSGControl();

//    mAc->getSimulationControl()->setStepSize(0.015);
//    mAc->getSimulationControl()->setNumFEMCorrectionIterations(3);
//    mAc->getSimulationControl()->setMaxNumConstraintSolverIterations(5);


    // Floor
    {
        SGLeafNode* floor = mAc->getSGControl()->createBox(
                    "Floor",
                    mAc->getSGControl()->getSceneGraph()->getRoot(),
                    Vector(0.0, -1.2, 0.0), 200, 2.0, 200, true);

        floor->getData()->getRenderModel()->setAppearances(
                    std::make_shared<Appearances>(
                        Appearance::createAppearanceFromColor(
        {0.8f, 0.8f, 0.8f, 1.0f})));

        mAc->getSGControl()->createRigidBody(floor->getData(), 1.0, true);
        mAc->getSGControl()->createCollidable(floor->getData());
    }

    // Brick Wall
//    {
//        double brickWidth = 0.5;
//        double brickHeight = 0.3;
//        double brickLength = 0.7;
//        size_t rows = 12;
//        size_t columns = 8;
//        double margin = 0.05; // distance between each brick
//        double wallLength = brickLength * columns;
//        for (size_t r = 0; r < rows; ++r)
//        {
//            double rowOffset = (r % 2) * brickLength / 2.0 - wallLength / 2.0;

//            for (size_t c = 0; c < columns; ++c)
//            {
//                SGLeafNode* brick = sg->createLeafNode(
//                            "Brick", sg->getSceneGraph()->getRoot(),
//                            std::make_shared<Polygon2D>(GeometricDataFactory::create2DBox(brickLength, brickWidth, brickHeight)),
//                            Eigen::Vector3d(rowOffset + c * (brickLength + margin), r * (brickHeight + margin), -2));

//                sg->createRigidBody(brick->getData(), 1.0);
//                sg->createCollidable(brick->getData());
//            }
//        }
//    }

    // Rigid Wall
    {
        SGLeafNode* rigidWall = sg->createLeafNode(
                    "Rigid Wall", sg->getSceneGraph()->getRoot(),
                    std::make_shared<Polygon2D>(GeometricDataFactory::create2DBox(1.0, 60.0, 20.0)),
                    Eigen::Vector3d(80.0, 10.0, 0.0));
        sg->createRigidBody(rigidWall->getData(), 1.0, true);
        sg->createCollidable(rigidWall->getData());
    }

    std::vector<std::array<float, 4>> colors;
    colors.push_back({0.0f, 0.0f, 0.0f, 1.0f});
    colors.push_back({1.0f, 0.0f, 0.0f, 1.0f});
    colors.push_back({0.0f, 1.0f, 0.0f, 1.0f});
    colors.push_back({0.0f, 0.0f, 1.0f, 1.0f});
    colors.push_back({1.0f, 1.0f, 0.0f, 1.0f});
    colors.push_back({1.0f, 0.0f, 1.0f, 1.0f});
    colors.push_back({0.0f, 1.0f, 1.0f, 1.0f});
    colors.push_back({1.0f, 1.0f, 1.0f, 1.0f});

//    int countX = 3;
//    int countY = 4;
//    int countZ = 2;

//    int countX = 2;
//    int countY = 1;
//    int countZ = 1;

    int countX = 9;
    int countY = 1;
    int countZ = 3;

    int total = countX * countY * countZ;

    for (int carIndex = 0; carIndex < total; ++carIndex)
    {
        int r = std::floor(carIndex / (countX * countZ));
        int c = carIndex % countX;
        int z = static_cast<int>(std::floor(static_cast<double>(carIndex) / countX)) % countZ;

        Eigen::Affine3d transformation;
        if (c >= countX / 2)
        {
            transformation =
                    Eigen::Affine3d::Identity() *
                    Eigen::Translation3d(-20.0 + 3.0 + (r % 2) + 9.0 * c, /*4.0 +*/ 1.0 + 2.0 * r, z * 2.0 + 1.5) *
                    Eigen::AngleAxisd(180.0 * M_PI / 180.0, Eigen::Vector3d(0.0, 1.0, 0.0));
        }
        else
        {
            transformation =
                    Eigen::Affine3d::Identity() *
                    Eigen::Translation3d(-40.0 + 3.0 + (r % 2) + 9.0 * c, /*4.0 +*/ 1.0 + 2.0 * r, z * 2.0);
        }
    //            Eigen::Affine3d::Identity() *
    //            Eigen::Translation3d(0.0, 0.5, 0.0) *
    //            Eigen::AngleAxisd(-30.0 * M_PI / 180.0, Eigen::Vector3d(0.0, 0.0, -1.0));

        // Car
        SGChildrenNode* carNode = sg->createChildrenNode(sg->getSceneGraph()->getRoot(), "Car");

        // chasis
        std::shared_ptr<FEMObject> chasisSo;
        std::shared_ptr<Polygon3D> chasisPoly;
        std::shared_ptr<MeshInterpolatorFEM> interpolator;
        {
            SGLeafNode* mesh3d;

            bool simpleDeformable = true;

            if (!simpleDeformable)
            {
                // Chasis poly -> original .node and .ele files

                std::vector<File> tetFiles = {
                    File(QCoreApplication::applicationDirPath().toStdString() + "/assets/FEMFX/car-body-tets.node"),
                    File(QCoreApplication::applicationDirPath().toStdString() + "/assets/FEMFX/car-body-tets.ele")
                };
                mesh3d = static_cast<SGLeafNode*>(
                            sg->importFilesAsChild(tetFiles, carNode));

                chasisPoly = std::static_pointer_cast<Polygon3D>(
                            mesh3d->getData()->getGeometricData());

                chasisPoly->transform(transformation);
            }
            else
            {
                // Chasis poly -> surrounding bounding box as 3d box

                // 2d mesh for collision detection
                SGLeafNode* mesh2d = static_cast<SGLeafNode*>(
                            sg->importFileAsChild(
                                File(QCoreApplication::applicationDirPath().toStdString() + "/assets/FEMFX/car-body-tets-convex.obj"),
                                carNode, false));
                std::shared_ptr<Polygon2D> poly2 =
                        std::static_pointer_cast<Polygon2D>(
                            mesh2d->getData()->getGeometricData());

                mesh2d->getData()->getRenderModel()->setVisible(false);
                poly2->transform(transformation);
                poly2->update();
                poly2->updateBoundingBox();

                // mesh for visualization
                SGLeafNode* meshVis = static_cast<SGLeafNode*>(
                            sg->importFileAsChild(
                                File(QCoreApplication::applicationDirPath().toStdString() + "/assets/FEMFX/car-body-tets.obj"),
                                carNode, false));
                std::shared_ptr<Polygon2D> polyVis =
                        std::static_pointer_cast<Polygon2D>(
                            meshVis->getData()->getGeometricData());
                polyVis->transform(transformation);
                polyVis->update();
                meshVis->getData()->getRenderModel()->setAppearances(
                            std::make_shared<Appearances>(
                                Appearance::createAppearanceFromColor(
                colors[carIndex % colors.size()])));

                // mesh for FEM simulation: BoundingBox deformable
                const BoundingBox& bb = poly2->getBoundingBox();

                std::cout << "bb.size() = " << bb.size().transpose() << "\n";
                chasisPoly = std::make_shared<Polygon3D>(
                            GeometricDataFactory::create3DBox(
                                bb.size()(0), bb.size()(2), bb.size()(1)));
                chasisPoly->transform(Eigen::Affine3d::Identity() *
                                      Eigen::Translation3d(0.5 * (bb.max() + bb.min())));
                chasisPoly->update();
                mesh3d = sg->createLeafNode(
                            "Deforming Box", carNode, chasisPoly, Eigen::Vector3d::Zero());
                mesh3d->getData()->setVisible(false);
                mesh3d->getData()->getRenderModel()->setWireframeEnabled(true);

                interpolator =
                        std::static_pointer_cast<MeshInterpolatorFEM>(
                            mAc->getInterpolatorModule()->addInterpolator(
                                mesh3d, mesh2d, MeshInterpolator::Type::FEM));

                mAc->getInterpolatorModule()->addInterpolator(
                            mesh3d, meshVis, MeshInterpolator::Type::FEM);
            }

            ElasticMaterial material;
            material.setFromYoungsPoisson(4e+3, 0.45);
            material.setPlasticYield(0.005);
            material.setPlasticCreep(10);
            material.setPlasticMaxStrain(1.0);
            chasisSo = sg->createFEMObject(mesh3d->getData(), 20.0);
            std::shared_ptr<FEMObject> femObj =
                    std::static_pointer_cast<FEMObject>(
                        mesh3d->getData()->getSimulationObject());
            femObj->setElasticMaterial(material);

            if (!simpleDeformable)
            {
                sg->createCollidable(mesh3d->getData());
            }
            else
            {
                sg->createCollidable(mesh3d->getData(), interpolator);
            }
            mAc->getSimulationControl()->addCollisionGroup(chasisSo, carIndex);
        }

        // tires
        std::array<std::string, 4> tireMeshes =
        {
            QCoreApplication::applicationDirPath().toStdString() + "/assets/FEMFX/car-wheel0-tets",
            QCoreApplication::applicationDirPath().toStdString() + "/assets/FEMFX/car-wheel1-tets",
            QCoreApplication::applicationDirPath().toStdString() + "/assets/FEMFX/car-wheel2-tets",
            QCoreApplication::applicationDirPath().toStdString() + "/assets/FEMFX/car-wheel3-tets"
        };
        std::array<SGLeafNode*, 4> tires;

        bool rigidTires = true;
        //
        for (size_t i = 0; i < 4; ++i)
        {
            std::vector<File> tetFiles = {
                File(tireMeshes[i] + ".node"),
                File(tireMeshes[i] + ".ele")
            };

            // load mesh
            SGLeafNode* mesh3d = static_cast<SGLeafNode*>(
                        sg->importFilesAsChild(tetFiles, nullptr));
            tires[i] = mesh3d;
            std::shared_ptr<Polygon3D> poly3 = std::static_pointer_cast<Polygon3D>(
                        mesh3d->getData()->getGeometricData());

            SGLeafNode* wheelNode = static_cast<SGLeafNode*>(
                        sg->importFileAsChild(
                            File(QCoreApplication::applicationDirPath().toStdString() + "/assets/primitives/cylinder_triagulated.obj"),
                            carNode));
            wheelNode->getData()->getRenderModel()->setAppearances(
                        std::make_shared<Appearances>(
                            Appearance::createAppearanceFromColor(
            {0.1f, 0.1f, 0.1f, 1.0f})));
            wheelNode->getData()->getGeometricData()->transform(
                        transformation *
                        Eigen::Translation3d(poly3->calculateCenterVertex()) *
                        Eigen::AngleAxisd(90.0 * M_PI / 180.0, Eigen::Vector3d(1.0, 0.0, 0.0)) *
                        Eigen::Scaling(0.4));
            wheelNode->getData()->getGeometricData()->update();
            std::shared_ptr<Polygon2D> wheelPoly =
                    std::static_pointer_cast<Polygon2D>(
                        wheelNode->getData()->getGeometricData());

//            poly3->transform(transformation);
//            poly3->update();
//            static_cast<PolygonRenderModel*>(mesh3d->getData()->getRenderModelRaw())->setRenderFaceNormals(true);

            std::shared_ptr<SimulationObject> so;
            if (rigidTires)
            {
                so = sg->createRigidBody(wheelNode->getData(), 1.0);
            }
            else
            {
                ElasticMaterial material;
                material.setFromYoungsPoisson(1e+6, 0.49);
            //        material.setPlasticYield(0.5);
            //        material.setPlasticCreep(10);
            //        material.setPlasticMaxStrain(10);
                so = sg->createFEMObject(wheelNode->getData(), 10.0);
                std::shared_ptr<FEMObject> femObj =
                        std::static_pointer_cast<FEMObject>(
                            wheelNode->getData()->getSimulationObject());
                femObj->setElasticMaterial(material);
            }

            sg->createCollidable(wheelNode->getData());

            mAc->getSimulationControl()->addCollisionGroup(so, carIndex);

            // Attach each tire to the chasis

            // point on deformable

            wheelPoly->updateBoundingBox();
            const BoundingBox& bb = wheelPoly->getBoundingBox();
            Eigen::Vector3d wheelCenter = wheelPoly->calculateCenterVertex();

            for (int j = 0; j < 2; ++j)
            {
                Eigen::Vector3d pos;
                if (j == 0)
                    pos = Eigen::Vector3d(wheelCenter(0), wheelCenter(1), bb.min()(2));
                else
                    pos = Eigen::Vector3d(wheelCenter(0), wheelCenter(1), bb.max()(2));
                std::tuple<size_t, double, Eigen::Vector4d> tuple =
                        chasisPoly->findTetrahedron(pos);
                Eigen::Vector4d bary = std::get<2>(tuple);
                std::array<double, 4> baryArray = {bary(0), bary(1), bary(2), bary(3)};
                size_t elementId = std::get<0>(tuple);

                std::shared_ptr<BallJoint> joint = std::make_shared<BallJoint>(
                            SimulationPointRef(chasisSo.get(), chasisPoly.get(), baryArray, elementId),
                            SimulationPointRef(so.get(), wheelPoly.get(), pos - wheelCenter));

                mAc->getSimulationControl()->addConstraint(joint);
            }

            double direction = c >= countX / 2 ? 1 : -1;
            std::shared_ptr<RotationalMotor> motorForce =
                    std::make_shared<RotationalMotor>(
                        std::dynamic_pointer_cast<RigidBody>(so), nullptr,
                        Vector(0.0, 0.0, direction), 20.0);
            mAc->getSimulationControl()->addForce(motorForce);

        }

    }
}

void CarCrashDemo::unload()
{
}
