#include <GL/glew.h>

#include "ApplicationControl.h"
#include "main_window.h"
#include "SimulationControl.h"
#include "RenderModelManager.h"

#include <io/ImageLoader.h>
#include <io/MeshIO.h>
#include <rendering/Appearance.h>
#include <rendering/Appearances.h>
#include <rendering/Image.h>
#include <rendering/Renderer.h>
#include <rendering/Texture.h>
#include <rendering/TextureUtils.h>
#include <scene/data/geometric/GeometricDataFactory.h>
#include <scene/data/geometric/GeometricDataListener.h>
#include <scene/data/geometric/GeometricDataUtils.h>
#include <scene/data/geometric/MeshInterpolatorMeshMesh.h>
#include <scene/data/geometric/Polygon3D.h>
#include <scene/scene_graph/SGControl.h>
#include <scene/scene_graph/SGCore.h>
#include <scene/scene_graph/SGNodeVisitorFactory.h>
#include <scene/scene_graph/SGTraverserFactory.h>
#include <scene/model/MeshInterpolatorRenderModel.h>
#include <scene/model/ModelFactory.h>
#include <scene/model/PolygonRenderModel.h>
#include <scene/model/RenderModel.h>
#include <simulation/fem/FEMObject.h>
#include <simulation/fem/FEMSimulation.h>
#include <scene/data/simulation/FEMData.h>
#include <times/timing.h>
#include <ui/ModulesUIControl.h>
#include <ui/UIControl.h>
#include <modules/mesh_converter/MeshConverterModule.h>
#include <modules/mesh_converter/MeshCriteria.h>
#include <simulation/SimulationModule.h>
#include <modules/demo_loader/Demo.h>
#include <modules/demo_loader/DemoLoaderModule.h>
#include <demos/ChainDemo.h>
#include <demos/DoublePendulumDemo.h>
#include <demos/RotationalJointsDemo.h>
#include <demos/LineJointDemo.h>
#include <demos/MobileDemo.h>
#include <demos/PlaneJointDemo.h>
#include <demos/CarDemo.h>
#include <demos/InterpolationMeshMeshDemo.h>
#include <demos/TexturingDemo.h>
#include <demos/InterpolationFEMDemo.h>
#include <ui/scene_graph/SGUIControl.h>
#include <io/importers/OBJImporter.h>
#include <modules/geometry_info/GeometryInfoModule.h>


ApplicationControl::ApplicationControl()
{
}

ApplicationControl::~ApplicationControl()
{
//    finalizeModules();
}

void ApplicationControl::initiateApplication()
{
    // initiate all controls



    // Simulation Control
    mSimulationControl = std::make_shared<SimulationControl>();

    // UiControl
    mUiControl = std::make_shared<UIControl>();

    // SGControl
    mSGControl = std::make_shared<SGControl>();

    // SGUIControl
    mSGUIControl = std::make_shared<SGUIControl>();

    mRenderModelManager = std::make_shared<RenderModelManager>(this);

    mSimulationControl->setApplicationControl(this);

    // first initiate ui elements
    mUiControl->initialize(this);

    // initiates simulation
    mSimulationControl->initialize();

    mSGControl->initialize(this);

    mSGUIControl->initialize(mSGControl.get(), mUiControl.get());

    mUiControl->connectSignals();



    // how to move poly3 to the heap?

    // init starting scene

    // ***********************************************************
    // Create scene graph, leaf nodes and leaf data

    //mSimulationControl->initialize();


//    mSimulationControl->setSimulationRunning(false);

    mSimulationControl->startSimulationThread();
//    mSimulationControl->initializeSimulation();
//    mSimulationControl->setSimulationRunning(true);

//    mSimulationControl->simulationLoop();

    // add fem object to fem simulation, get fem simulation from simulation control

    // FEMSimulatoin::addFEMObject !!

    // old way of doing it:
//    Vectors vertices_out;
//    Faces outer_faces_out, faces_out;
//    Cells cells_out;

//    // convert Polygon2D to Polygon3D
//    MeshConverter::instance()->setCellRadiusEdgeRatio(30);
//    MeshConverter::instance()->setCellSize(0.3);
//    MeshConverter::instance()->generateMesh(so->getPositions(), so->getFaces(),
//                                               vertices_out, outer_faces_out, faces_out, cells_out);
//    so->setMesh(vertices_out, faces_out, outer_faces_out, cells_out);

    // Modules
    createModules();
    initiateModules();

    class EmptyDemo : public Demo
    {
    public:
        EmptyDemo()
        {

        }

        virtual std::string getName()
        {
            return "Empty Scene";
        }

        virtual void load()
        {

        }

        virtual void unload()
        {

        }

    };
    std::shared_ptr<Demo> emptyDemo = std::make_shared<EmptyDemo>();
    mDemoLoaderModule->addDemo(emptyDemo);

    class Example1Demo : public Demo
    {
    public:
        Example1Demo(ApplicationControl& _ac)
            : ac(_ac)
        {

        }

        virtual std::string getName()
        {
            return "Example Demo 1";
        }

        virtual void load()
        {
            ac.getSimulationControl()->setGravity(Vector::Zero());
            ac.getSimulationControl()->setNumFEMCorrectionIterations(0);

            double boxDim = 3.0;

            SGLeafNode* node1 = ac.mSGControl->createLeafNode(
                        "Box", ac.mSGControl->getSceneGraph()->getRoot(),
                        std::make_shared<Polygon3D>(
                            GeometricDataFactory::create3DBox(boxDim, boxDim, boxDim)),
                        Vector(0.0, 0.0, 0.0), true);

            // deformable
//                    MeshCriteria criteria(0.0, 0.0, 0.0, 0.15, 30, true);
//                    MeshCriteria criteria(0.0, 0.0, 0.0, 0.15, 30, false);
//                    ac.mSGControl->create3DGeometryFrom2D(node1, criteria, true);

            ac.mSGControl->createFEMObject(node1->getData());
        }

        virtual void unload()
        {

        }

        ApplicationControl& ac;
    };
    std::shared_ptr<Demo> example1Demo = std::make_shared<Example1Demo>(*this);
    mDemoLoaderModule->addDemo(example1Demo);

    class Example2Demo : public Demo
    {
    public:
        Example2Demo(ApplicationControl& _ac)
            : ac(_ac)
        {

        }

        virtual std::string getName()
        {
            return "Rigid Deformable Collision";
        }

        virtual void load()
        {
            // Floor
            SGLeafNode* node2 = ac.mSGControl->createBox("Floor", ac.mSGControl->getSceneGraph()->getRoot(),
                                                      /*Vector(-5, -2, -5)*/Vector(0.0, -1.5, 0.0), 6, 0.5, 6, true);

            ac.mSGControl->createRigidBody(node2->getData(), 1.0, true);
            ac.mSGControl->createCollidable(node2->getData());

            // some boxes:
            for (int r = 0; r < 4; ++r)
            {
                for (int c = 0; c < 4; ++c)
                {
                    if (r == 0)
                    {
                        MeshCriteria criteria(0.0, 0.0, 0.0, 0.0, 0.0);
                        SGLeafNode* node1 = ac.mSGControl->createBox("Box", ac.mSGControl->getSceneGraph()->getRoot(),
                                                                  Vector(-1 + 0.6 * c, -0.5 + 0.7 * r, 0.0), 0.5, 0.5, 0.5, true);
                        ac.mSGControl->create3DGeometryFrom2D(node1, criteria);
                        ac.mSGControl->createFEMObject(node1->getData());
//                        ac.mSGControl->createRigidBody(node1->getData(), 1.0, true);
                        ac.mSGControl->createCollidable(node1->getData());
                    }
                    else
                    {
                        SGLeafNode* node1 = ac.mSGControl->createBox("Box", ac.mSGControl->getSceneGraph()->getRoot(),
                                                                  Vector(-1 + 0.6 * c, -0.5 + 0.7 * r, 0.0), 0.5, 0.5, 0.5, true);
                        ac.mSGControl->createRigidBody(node1->getData(), 1.0, false);
                        ac.mSGControl->createCollidable(node1->getData());
                    }

                }
            }
        }

        virtual void unload()
        {

        }

        ApplicationControl& ac;
    };

    mDemoLoaderModule->addDemo(std::make_shared<Example2Demo>(*this));
    mDemoLoaderModule->addDemo(std::make_shared<DoublePendulumDemo>(*this));
    mDemoLoaderModule->addDemo(std::make_shared<ChainDemo>(*this));
    mDemoLoaderModule->addDemo(std::make_shared<LineJointDemo>(*this));
    mDemoLoaderModule->addDemo(std::make_shared<PlaneJointDemo>(*this));
    mDemoLoaderModule->addDemo(std::make_shared<RotationalJointsDemo>(*this));

    std::shared_ptr<MobileDemo> mobileDemo = std::make_shared<MobileDemo>(*this);
    mDemoLoaderModule->addDemo(mobileDemo);

    std::shared_ptr<CarDemo> carDemo = std::make_shared<CarDemo>(*this);
    mDemoLoaderModule->addDemo(carDemo);

    std::shared_ptr<InterpolationMeshMeshDemo> interpolationDemo =
            std::make_shared<InterpolationMeshMeshDemo>(this);
    mDemoLoaderModule->addDemo(interpolationDemo);
    mDemoLoaderModule->addDemo(std::make_shared<InterpolationFEMDemo>(this));
    mDemoLoaderModule->addDemo(std::make_shared<TexturingDemo>(this));

    mDemoLoaderModule->loadDemo(emptyDemo);

}

void ApplicationControl::createModules()
{
    mDemoLoaderModule = std::make_shared<DemoLoaderModule>();
    mModules.push_back(mDemoLoaderModule);
    mModules.push_back(std::make_shared<SimulationModule>());
    mModules.push_back(std::make_shared<MeshConverterModule>());
    mModules.push_back(std::make_shared<GeometryInfoModule>());
}

void ApplicationControl::initiateModules()
{
    for (const std::shared_ptr<Module>& m : mModules)
    {
//        m->init(mUiControl->);
        m->init(this);
        // parent = tab widget required
//        m->initUI(mUiControl->getModulesWidget());
        mUiControl->getModulesUIControl()->addModule(m.get());
    }
}

void ApplicationControl::finalizeModules()
{
    for (const std::shared_ptr<Module>& m : mModules)
    {
        m->finalize();
        mUiControl->getModulesUIControl()->removeModule(m.get());
    }
}

void ApplicationControl::addModulesTabs()
{
    for (const std::shared_ptr<Module>& m : mModules)
    {
        mUiControl->getModulesUIControl()->addModule(m.get());
    }
}

void ApplicationControl::removeModulesTabs()
{
    for (const std::shared_ptr<Module>& m : mModules)
    {
        mUiControl->getModulesUIControl()->removeModule(m.get());
    }
}

void ApplicationControl::handlePreRenderingStep()
{

    mRenderModelManager->updateAllRenderModels();

    // Update scene graph
    if (mSGControl)
    {
//        SGNodeVisitor* printerVisitor = SGNodeVisitorFactory::createPrinterVisitor(std::cout);
//        SGTraverserFactory::createDefaultSGTraverser(mSceneGraph->getRoot())
//                .traverse(*printerVisitor);

        // TODO: also add selection models to renderer here?
        SGTraverser traverser = SGTraverserFactory::createDefaultSGTraverser(
                    mSGControl->getSceneGraph()->getRoot());
        class UpdateSOVisitor : public SGNodeVisitor
        {
        public:
            UpdateSOVisitor(ApplicationControl& _ac)
                : ac(_ac)
            {

            }
            virtual void visit(SGChildrenNode* /*childrenNode*/)
            {

            }
            virtual void visit(SGLeafNode* leafNode)
            {
                // add render models to renderer that are not already added
                RenderModel* renderModel = leafNode->getData()->getRenderModelRaw();
                if (renderModel != nullptr)
                {
                    if (!renderModel->isAddedToRenderer())
                    {
                        Renderer* renderer = ac.mUiControl->getRenderer();
                        renderModel->addToRenderer(renderer);
                        renderModel->setAddedToRenderer(true);
                    }

                    // update render models that always want to be updated
                    if (renderModel->isAlwaysUpdate())
                        renderModel->update();

                    // update render models when their simulation objects think they need an update
//                    if (SimulationObject* so = leafNode->getData()->getSimulationObjectRaw())
//                    {
                        // only render model of leaf datas are called
                        // there must be some kind of vector of rende models that can be called independently
                        // of scene nodes or each linear force must be added to the scene graph.
                        // The scene graph option seems to be the better one because it allows the user to
                        // interact with the linear force
                        leafNode->getData()->getRenderModel()->update();
//                    }

                }
            }
            ApplicationControl& ac;
        } nodeVisitor(*this);

        traverser.traverse(nodeVisitor);
    }
}

void ApplicationControl::onExit()
{
    PRINT_TIMING
}

SGControl* ApplicationControl::getSGControl()
{
    return mSGControl.get();
}

SGUIControl* ApplicationControl::getSGUIControl()
{
    return mSGUIControl.get();
}

SimulationControl* ApplicationControl::getSimulationControl()
{
    return mSimulationControl.get();
}

UIControl* ApplicationControl::getUIControl()
{
    return mUiControl.get();
}

RenderModelManager* ApplicationControl::getRenderModelManager()
{
    return mRenderModelManager.get();
}
