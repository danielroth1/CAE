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
#include <scene/data/geometric/MeshInterpolationManager.h>
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
#include <demos/PlaneJointDemo.h>
#include <demos/CarDemo.h>
#include <demos/CarDemo2.h>
#include <demos/TexturingDemo.h>
#include <demos/InterpolationFEMDemo.h>
#include <demos/FallingObjectsDemo.h>
#include <demos/CubeWallDemo.h>
#include <demos/ConstrainedDeformableDemo.h>
#include <demos/InterpolationFEMCollisionDemo.h>
#include <demos/InterpolatorCreationDemo.h>
#include <demos/PlasticityDemo.h>
#include <demos/CarCrashDemo.h>
#include <ui/scene_graph/SGUIControl.h>
#include <io/importers/OBJImporter.h>
#include <modules/geometry_info/GeometryInfoModule.h>
#include <modules/interpolator/InterpolatorModule.h>


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

    // MeshInterpolationManager
    mMeshInterpolationManager = std::make_shared<MeshInterpolationManager>(
                mUiControl->getRenderer());



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
            ac.mSGControl->createCollidable(node1->getData());
        }

        virtual void unload()
        {

        }

        ApplicationControl& ac;
    };
    mDemoLoaderModule->addDemo(std::make_shared<Example1Demo>(*this));
    mDemoLoaderModule->addDemo(std::make_shared<CubeWallDemo>(this, true));
    mDemoLoaderModule->addDemo(std::make_shared<CubeWallDemo>(this, false));
    mDemoLoaderModule->addDemo(std::make_shared<DoublePendulumDemo>(*this));
    mDemoLoaderModule->addDemo(std::make_shared<ChainDemo>(*this));
    mDemoLoaderModule->addDemo(std::make_shared<LineJointDemo>(*this));
    mDemoLoaderModule->addDemo(std::make_shared<PlaneJointDemo>(*this));
    mDemoLoaderModule->addDemo(std::make_shared<RotationalJointsDemo>(*this));

    mDemoLoaderModule->addDemo(std::make_shared<CarDemo>(*this));
    mDemoLoaderModule->addDemo(std::make_shared<CarCrashDemo>(this));
    mDemoLoaderModule->addDemo(std::make_shared<CarDemo2>(*this));

    mDemoLoaderModule->addDemo(std::make_shared<InterpolationFEMDemo>(this, true, MeshInterpolator::Type::MESH_MESH));
    mDemoLoaderModule->addDemo(std::make_shared<InterpolationFEMDemo>(this, false, MeshInterpolator::Type::MESH_MESH));
    mDemoLoaderModule->addDemo(std::make_shared<InterpolationFEMDemo>(this, true, MeshInterpolator::Type::FEM));
    mDemoLoaderModule->addDemo(std::make_shared<InterpolationFEMDemo>(this, false, MeshInterpolator::Type::FEM));
    mDemoLoaderModule->addDemo(std::make_shared<InterpolationFEMCollisionDemo>(*this));
    mDemoLoaderModule->addDemo(std::make_shared<InterpolatorCreationDemo>(this));
    mDemoLoaderModule->addDemo(std::make_shared<TexturingDemo>(this));
    mDemoLoaderModule->addDemo(std::make_shared<FallingObjectsDemo>(this, "Falling Objects (Rigid)", true));
    mDemoLoaderModule->addDemo(std::make_shared<FallingObjectsDemo>(this, "Falling Objects (Deformable)", false));

    mDemoLoaderModule->addDemo(std::make_shared<ConstrainedDeformableDemo>(this));

    mDemoLoaderModule->addDemo(std::make_shared<PlasticityDemo>(this));

    mDemoLoaderModule->loadDemo(emptyDemo);

}

void ApplicationControl::createModules()
{
    mDemoLoaderModule = std::make_shared<DemoLoaderModule>();
    mModules.push_back(mDemoLoaderModule);
    mModules.push_back(std::make_shared<SimulationModule>());
    mModules.push_back(std::make_shared<MeshConverterModule>());
    mInterpolatorModule = std::make_shared<InterpolatorModule>();
    mModules.push_back(mInterpolatorModule);
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

                    // Ignore simulation objects because their models are
                    // updated in the simulation thread at the end of
                    // each time step.
                    if (leafNode->getData()->getSimulationObjectRaw() != nullptr)
                    {
                        return;
                    }

                    // update render models that always want to be updated
                    if (renderModel->isAlwaysUpdate())
                    {
                        renderModel->update();
                    }
                    else
                    {
                        // TODO: for the moment, controlling if data needs
                        // to be updated or not is done in the specific
                        // render models, so, RenderModel::update() is called
                        // for every object.
                        renderModel->update();
                    }


                }
            }
            ApplicationControl& ac;
        } nodeVisitor(*this);

        traverser.traverse(nodeVisitor);
    }
}

void ApplicationControl::updateSimulationObjects()
{
    if (mSGControl)
    {
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
                RenderModel* renderModel = leafNode->getData()->getRenderModelRaw();
                if (renderModel != nullptr)
                {
                    if (leafNode->getData()->getSimulationObjectRaw() != nullptr)
                    {
                        renderModel->update();
                    }
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

InterpolatorModule* ApplicationControl::getInterpolatorModule()
{
    return mInterpolatorModule.get();
}

MeshInterpolationManager* ApplicationControl::getMeshInterpolationManager()
{
    return mMeshInterpolationManager.get();
}
