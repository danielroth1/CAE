#include "ApplicationControl.h"
#include "RenderModelManager.h"
#include "SimulationControl.h"

#include <modules/mesh_converter/MeshConverter.h>
#include <chrono>
#include <memory>
#include <QDebug>
#include <simulation/fem/FEMSimulation.h>
#include <simulation/constraints/ConstraintVisitor.h>
#include <simulation/constraints/LinearForce.h>
#include <simulation/constraints/Truncation.h>
#include <simulation/Simulation.h>
#include <simulation/SimulationControlListener.h>
#include <simulation/SimulationObject.h>
#include <simulation/SimulationObjectVisitor.h>
#include <simulation/SimulationThread.h>
#include <simulation/fem/FEMObject.h>
#include <simulation/fem/FEMObjectUtil.h>
#include <multi_threading/StepperThread.h>
#include <ui/selection/Selection.h>
#include <ui/UIControl.h>
#include <simulation/collision_detection/CollisionControl.h>
#include <simulation/collision_detection/CollisionManager.h>
#include <simulation/models/LinearForceRenderModel.h>
#include <simulation/rigid/RigidBody.h>
#include <simulation/rigid/RigidCollisionSolver.h>
#include <simulation/rigid/RigidSimulation.h>
#include <scene/data/geometric/Polygon.h>
#include <scene/data/geometric/Polygon3D.h>
#include "glwidget.h"

using namespace Eigen;

SimulationControl::SimulationControl()
{
//    m_main_window = main_window;
//    m_gl_widget = m_main_window->getGlWidget();
//    m_update_interval = 500 ; // update after every 50 milliseconds
//    mUpdateThread = new std::thread(&SimulationControl::simulationLoop, this);
//    m_surface.create();
    mFEMSimulation = nullptr;
    mFEMSimulationProxy = nullptr;
    mSimulationThread = new SimulationThread(this);
    mDomain = new Domain();
    mPaused = false;
    mRigidCollisionSolver = std::make_unique<RigidCollisionSolver>();
    mSimulationThread->addDomain(mDomain);

    mStepSize = 0.01;
    mNumFEMCorrectionIterations = 0;
}

SimulationControl::~SimulationControl()
{
    delete mSimulationThread;
    delete mDomain;
}

void SimulationControl::setApplicationControl(ApplicationControl* ac)
{
    mAc = ac;
    mUiControl = mAc->getUIControl();
}

void SimulationControl::handleFileInput(int /*argc*/, char* /*argv*/[])
{
//    qDebug() << argc << "\n";
//    if (argc > 1)
//    {
//        // first parameter = input file name
//        char* filename = argv[1];
//        MeshConverter::getInstance().readInFile(filename);
    //    }
}

bool SimulationControl::addListener(SimulationControlListener* listener)
{
    auto it = std::find(mListeners.begin(), mListeners.end(), listener);
    if (it == mListeners.end())
    {
        mListeners.push_back(listener);
        return true;
    }
    return false;
}

bool SimulationControl::removeListener(SimulationControlListener* listener)
{
    auto it = std::find(mListeners.begin(), mListeners.end(), listener);
    if (it != mListeners.end())
    {
        mListeners.erase(it);
        return true;
    }
    return false;
}

void SimulationControl::actForce(Selection* /*selection*/, Vector /*force*/)
{

}

void SimulationControl::clearTruncations()
{
    mFEMSimulationProxy->clearTruncation();
}

void SimulationControl::addTruncations(FEMObject* femObject, std::vector<ID> vectorIDs)
{
    mFEMSimulationProxy->addTruncation(femObject, vectorIDs);
}

void SimulationControl::addLinearForce(
        std::shared_ptr<LinearForce> linearForce)
{
    for (std::shared_ptr<SimulationProxy>& simulationProxy : mSimulationProxies)
    {
        simulationProxy->addLinearForce(linearForce);
    }

    std::shared_ptr<LinearForceRenderModel> renderModel =
            std::make_shared<LinearForceRenderModel>(linearForce);

    mAc->getRenderModelManager()->addRenderModelByObject(linearForce, renderModel);
}

void SimulationControl::addLinearForce(
        SimulationObject* source, ID sourceVector,
        SimulationObject* target, ID targetVector,
        double strength)
{
    // create linear force
    std::shared_ptr<LinearForce> lf = std::make_shared<LinearForce>(
                SimulationPointRef(source, sourceVector),
                SimulationPointRef(target, targetVector),
                strength);

    // add to simulation
    for (std::shared_ptr<SimulationProxy>& simulationProxy : mSimulationProxies)
    {
        simulationProxy->addLinearForce(
                    static_pointer_cast<LinearForce>(lf->shared_from_this()));
    }


    std::cout << "count = " << lf.use_count() <<"\n";

    // create render model and add to renderer
    std::shared_ptr<LinearForceRenderModel> renderModel =
            std::make_shared<LinearForceRenderModel>(lf);

    mAc->getRenderModelManager()->addRenderModelByObject(lf, renderModel);
}

void SimulationControl::initialize()
{
//    std::unique_ptr<SimulationObject> so = std::unique_ptr<SimulationObject>(new SimulationObject());

    mCollisionControl = std::make_shared<CollisionControl>(mDomain, mUiControl, mAc->getRenderModelManager());
    mCollisionManager = mCollisionControl->getCollisionManager();
    mCollisionManagerProxy = std::make_shared<CollisionManagerProxy>(mCollisionManager.get());

    // initialize simulation
    // create FEMObject
    mFEMSimulation = std::make_shared<FEMSimulation>(mDomain, mStepSize, mCollisionManager);
    mFEMSimulationProxy = std::make_shared<FEMSimulationProxy>(mFEMSimulation.get());

    mRigidSimulation = std::make_shared<RigidSimulation>(mDomain, mStepSize, mCollisionManager);
    mRigidSimulationProxy = std::make_shared<RigidSimulationProxy>(mRigidSimulation.get());

    addSimulation(mFEMSimulation, mFEMSimulationProxy);
    addSimulation(mRigidSimulation, mRigidSimulationProxy);

    // create opengl shared context
//    QOpenGLContext* context = new QOpenGLContext;
//    context->setShareContext(m_gl_widget->context());
//    context->create();
//    context->makeCurrent(&m_surface);

//    context->doneCurrent();
//    delete context;

    mSimulationThread->start(); // only initilaize after everything is addeddd

    qDebug() << "finished";
//    m_gl_widget->setDrawable(mFEMSimulation->getRenderSimulation());

}

void SimulationControl::startSimulationThread()
{
    // start simulation loop
    mUpdateInterval = 20 ; // update after every 50 milliseconds
}

void SimulationControl::setSimulationPaused(bool paused)
{
    mPaused = paused;
    mSimulationThread->setPaused(paused);
}

bool SimulationControl::isSimulationPaused()
{
    return mPaused;
}

//void SimulationControl::repaint()
//{
//    if (mUiControl)
//        mUiControl->repaint();
//}

std::shared_ptr<FEMSimulation> SimulationControl::getFEMSimulation()
{
    return mFEMSimulation;
}

void SimulationControl::addSimulationObject(std::shared_ptr<SimulationObject> so)
{
    mSimulationObjects.push_back(so);

    class AddSimulationObjectVisitor : public SimulationObjectVisitor
    {
    public:
        AddSimulationObjectVisitor(SimulationControl& _sc)
            : sc(_sc)
        {

        }

        virtual void visit(FEMObject& femObject)
        {
            //static_cast<std::shared_ptr<SimulationObject>>(so);
            // TODO: addFEMObject should take a shared_ptr,
            // create shared_ptr with femObject.shared_from_this()
            sc.mFEMSimulationProxy->addFEMObject(
                        static_pointer_cast<FEMObject>(femObject.shared_from_this()));

            for (SimulationControlListener* l : sc.mListeners)
                l->onSimulationObjectAdded(&femObject);

        }

        virtual void visit(SimulationPoint& /*sp*/)
        {

        }

        virtual void visit(RigidBody& rigidBody)
        {
            sc.mRigidSimulationProxy->addRigidBody(
                        static_pointer_cast<RigidBody>(rigidBody.shared_from_this()));

            for (SimulationControlListener* l : sc.mListeners)
                l->onSimulationObjectAdded(&rigidBody);

        }

        SimulationControl& sc;
    } visitor(*this);
    so->accept(visitor);
}

void SimulationControl::removeSimulationObject(const std::shared_ptr<SimulationObject>& so)
{
    auto it = std::find(mSimulationObjects.begin(), mSimulationObjects.end(), so);
    if (it == mSimulationObjects.end())
    {
        return;
    }
    else
    {
        mSimulationObjects.erase(it);
    }

    // Remove all linear forces that reference the simulation object
    for (size_t i = 0; i < mSimulations.size(); ++i)
    {
        const std::shared_ptr<Simulation>& simulation = mSimulations[i];
        for (const std::shared_ptr<LinearForce>& lf : simulation->getLinearForces())
        {
            if (lf->references(so.get()))
            {
                mSimulationProxies[i]->removeLinearForce(lf.get());
                mAc->getRenderModelManager()->removeRenderModelByObject(lf);
            }
        }
    }

    class RemoveSimulationObjectVisitor : public SimulationObjectVisitor
    {
    public:
        RemoveSimulationObjectVisitor(SimulationControl& _sc)
            : sc(_sc)
        {

        }

        virtual void visit(FEMObject& femObject)
        {
            sc.mFEMSimulationProxy->removeFEMObject(&femObject);

            for (SimulationControlListener* l : sc.mListeners)
                l->onSimulationObjectRemoved(&femObject);
        }

        virtual void visit(SimulationPoint& /*sp*/)
        {

        }

        virtual void visit(RigidBody& rigidBody)
        {
            sc.mRigidSimulationProxy->removeRigidBody(&rigidBody);

            for (SimulationControlListener* l : sc.mListeners)
                l->onSimulationObjectRemoved(&rigidBody);
        }

        SimulationControl& sc;
    } visitor(*this);
    so->accept(visitor);

    mCollisionManagerProxy->removeSimulationObject(so);
}

void SimulationControl::clearSimulationObjects()
{
    // create a temporary copy because the original vector
    // is adapted while iterating.
    std::vector<std::shared_ptr<SimulationObject>> simulationObjects = mSimulationObjects;
    for (const std::shared_ptr<SimulationObject>& so : simulationObjects)
    {
        removeSimulationObject(so);
    }
}

void SimulationControl::addConstraint(std::shared_ptr<Constraint> so)
{
    class CV : public ConstraintVisitor
    {
    public:
        CV(SimulationControl& _sc)
            : sc(_sc)
        {

        }

        virtual void visit(LinearForce* linearForce)
        {
            for (std::shared_ptr<SimulationProxy>& simulationProxy : sc.mSimulationProxies)
            {
                simulationProxy->addLinearForce(
                            static_pointer_cast<LinearForce>(linearForce->shared_from_this()));
            }

        }

        SimulationControl& sc;
    } visitor(*this);
    so->accept(visitor);
}

void SimulationControl::removeConstraint(Constraint* so)
{
    class CV : public ConstraintVisitor
    {
    public:
        CV(SimulationControl& _sc)
            : sc(_sc)
        {

        }

        virtual void visit(LinearForce* linearForce)
        {
            for (std::shared_ptr<SimulationProxy>& simulationProxy : sc.mSimulationProxies)
            {
                simulationProxy->removeLinearForce(linearForce);
            }

        }

        SimulationControl& sc;
    } visitor(*this);
    so->accept(visitor);
}

void SimulationControl::addCollisionObject(std::shared_ptr<SimulationObject> so)
{
    class CollisionObjectAdder : public SimulationObjectVisitor
    {
    public:
        CollisionObjectAdder(SimulationControl& _sc)
            : sc(_sc)
        {

        }

        virtual void visit(FEMObject& femObject)
        {
            sc.mCollisionManagerProxy->addSimulationObject(
                        femObject.shared_from_this(),
                        femObject.getPolygon());
        }

        virtual void visit(SimulationPoint& /*sp*/)
        {
            std::cout << "Can not craete CollisionObject from SimulationPoint\n";
        }

        virtual void visit(RigidBody& rigidBody)
        {
            sc.mCollisionManagerProxy->addSimulationObject(
                        rigidBody.shared_from_this(),
                        rigidBody.getPolygon());
        }

        SimulationControl& sc;
    } v(*this);
    so->accept(v);
}

void SimulationControl::removeCollisionObject(const std::shared_ptr<SimulationObject>& so)
{
    mCollisionManagerProxy->removeSimulationObject(so);
}

void SimulationControl::initializeSimulation()
{
    mFEMSimulation->initialize();
    mRigidSimulation->initialize();
}

void SimulationControl::step()
{
    // Solver step algorithm
    // -> iterate for as long as all collisions are resolved
    // Collision Resolution
    // for each collision
    //      calculate collision correction impulse that makes
    //      the collision points reach their desired speed.

    // Collision:
    //      collision point a, b
    //      desired velocity after collision
    //      sum of impulses

    mRigidSimulation->initializeStep();
    mFEMSimulation->initializeStep();

    mRigidSimulation->solve();
    mFEMSimulation->solve(true); // x + x^{FEM}, v + v^{FEM}

     // collision detection on x + x^{FEM}
    if (mCollisionManager->collideAll())
    {
        // create collision constraints w.r.t. x + x^{FEM}
        mRigidCollisionSolver->initialize(
                    mCollisionManager->getCollider()->getCollisions(),
                    mStepSize,
                    0.2, // Restitution (bounciness factor)
                    1e-5); // maxCollisionDistance

        // Revert the illegal state
        mRigidSimulation->revertPositions();
        mFEMSimulation->revertPositions(); // x, v + v^{FEM}

        mRigidCollisionSolver->solveConstraints(300, 1e-5); // x, v + v^{FEM} + v^{col}

        for (int i = 0; i < mNumFEMCorrectionIterations; ++i)
        {
            mFEMSimulation->revertSolverStep(); // x, v + v^{col}

            // solve on x, v + v^{col}
            mFEMSimulation->solve(true); // x + x^{FEM}, v + v^{FEM}

            mFEMSimulation->revertPositions(); // x, v + v^{FEM}

            mRigidCollisionSolver->solveConstraints(300, 1e-5); // x, v + v^{FEM} + v^{col}
        }
        // x, v + v^{FEM} + v^{col}

        mRigidSimulation->integratePositions();
        mFEMSimulation->integratePositions(); // x + x^{FEM} + x^{col}, v + v^{FEM} + v^{col}
    }

    mRigidSimulation->applyDamping();
    mFEMSimulation->applyDamping();

    mRigidSimulation->publish();
    mFEMSimulation->publish();

    handleAfterStep();
}

std::vector<std::shared_ptr<Simulation>>& SimulationControl::getSimulations()
{
    return mSimulations;
}

void SimulationControl::setCollisionRenderingLevel(int level)
{
    mCollisionControl->setBvhRenderLevel(level);
}

void SimulationControl::setBVHCollisionRenderingEnabled(bool enabled)
{
    mCollisionControl->setBvhRenderingEnables(enabled);
}

Domain* SimulationControl::getDomain()
{
    return mDomain;
}

void SimulationControl::addSimulation(const std::shared_ptr<Simulation>& simulation,
                                      const std::shared_ptr<SimulationProxy>& proxy)
{
    simulation->initialize();
    mSimulations.push_back(simulation);
    mSimulationProxies.push_back(proxy);
}

void SimulationControl::removeSimulation(std::shared_ptr<Simulation>& simulation)
{
    auto it = std::find(mSimulations.begin(), mSimulations.end(), simulation);
    if (it != mSimulations.end())
    {
        mSimulations.erase(it);
    }
    // TODO: also remove the corresponding simulation proxy! If not, there will
    // be a guaranteed segmentation fault when accessing it. But realisticly,
    // when are simulations even removed?
}

void SimulationControl::handleAfterStep()
{
    mCollisionManager->updateAll();
//    repaint();
}
