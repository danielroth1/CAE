#include "ApplicationControl.h"
#include "RenderModelManager.h"
#include "SimulationControl.h"

#include <modules/mesh_converter/MeshConverter.h>
#include <chrono>
#include <memory>
#include <QDebug>
#include <simulation/fem/FEMSimulation.h>
#include <simulation/constraints/ConstraintVisitor.h>
#include <simulation/forces/LinearForce.h>
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
#include <simulation/ImpulseConstraintSolver.h>
#include <simulation/rigid/RigidSimulation.h>
#include <scene/data/geometric/Polygon.h>
#include <scene/data/geometric/Polygon3D.h>
#include <times/timing.h>
#include "glwidget.h"

using namespace Eigen;

SimulationControl::SimulationControl()
{
//    m_main_window = main_window;
//    m_gl_widget = m_main_window->getGlWidget();
//    m_update_interval = 500 ; // update after every 50 milliseconds
//    mUpdateThread = new std::thread(&SimulationControl::simulationLoop, this);
//    m_surface.create();
    mProxy = std::make_shared<SimulationControlProxy>(this);
    mFEMSimulation = nullptr;
    mFEMSimulationProxy = nullptr;
    mSimulationThread = new SimulationThread(this);
    mDomain = new Domain();
    mPaused = false;
    mGravity = Eigen::Vector(0.0, -9.81, 0.0);
    mImpulseConstraintSolver = std::make_unique<ImpulseConstraintSolver>();
    mSimulationThread->addDomain(mDomain);

    mStepSize = 0.01;
    mNumFEMCorrectionIterations = 5;
    mMaxNumConstraintSolverIterations = 5;
    mMaxConstraintError = 1e-5;
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
    addForce(linearForce);

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
    addForce(lf);

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
    mCollisionManager->setInvertNormalsIfNecessary(true);

    // initialize simulation
    // create FEMObject
    mFEMSimulation = std::make_shared<FEMSimulation>(mDomain, mCollisionManager);
    mFEMSimulationProxy = std::make_shared<FEMSimulationProxy>(mFEMSimulation.get());

    mRigidSimulation = std::make_shared<RigidSimulation>(mDomain, mCollisionManager);
    mRigidSimulationProxy = std::make_shared<RigidSimulationProxy>(mRigidSimulation.get());

    mFEMSimulation->initialize();
    mRigidSimulation->initialize();

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

void SimulationControl::performSingleStep()
{
    if (mPaused)
    {
        mProxy->step();
    }
}

void SimulationControl::setGravity(Vector gravity)
{
    mGravity = gravity;
}

Vector SimulationControl::getGravity() const
{
    return mGravity;
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

void SimulationControl::setStepSize(double stepSize)
{
    mStepSize = stepSize;
    mSimulationThread->setTimeStepSize(stepSize);
}

double SimulationControl::getStepSize() const
{
    return mStepSize;
}

void SimulationControl::setMaxNumConstraintSolverIterations(
        int numConstraintSolverIterations)
{
    mMaxNumConstraintSolverIterations = numConstraintSolverIterations;
}

int SimulationControl::getMaxNumConstraintSolverIterations() const
{
    return mMaxNumConstraintSolverIterations;
}

void SimulationControl::setMaxConstraintError(double maxConstraintError)
{
    mMaxConstraintError = maxConstraintError;
}

double SimulationControl::getMaxConstraintError() const
{
    return mMaxConstraintError;
}

void SimulationControl::setNumFEMCorrectionIterations(int correctionIterations)
{
    mNumFEMCorrectionIterations = correctionIterations;
}

int SimulationControl::getNumFEMCorrectionIterations() const
{
    return mNumFEMCorrectionIterations;
}

void SimulationControl::setInvertNormalsIfNecessary(bool invertNormalsIfNecessary)
{
    mCollisionManagerProxy->setInvertNormalsIfNecessary(invertNormalsIfNecessary);
}

bool SimulationControl::getInvertNormalsIfNecessary() const
{
    return mCollisionManager->getInvertNormalsIfNecessary();
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

void SimulationControl::addSimulationObject(
        const std::shared_ptr<SimulationObject>& so)
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
    std::vector<std::shared_ptr<Force>> forcesToBeRemoved =
            retrieveReferencingMechanicalProperties(so, mForces);
    for (const std::shared_ptr<Force>& lf : forcesToBeRemoved)
    {
        mAc->getRenderModelManager()->removeRenderModelByObject(lf);
        removeForce(lf);
    }

    std::vector<std::shared_ptr<Constraint>> constraintsToBeRemoved =
            retrieveReferencingMechanicalProperties(so, mConstraints);
    for (const std::shared_ptr<Constraint>& c : constraintsToBeRemoved)
    {
        removeConstraint(c);
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

void SimulationControl::addForce(const std::shared_ptr<Force>& f)
{
    mProxy->addForceSlot(f);
}

void SimulationControl::removeForce(const std::shared_ptr<Force>& f)
{
    mProxy->removeForceSlot(f);
}

void SimulationControl::addConstraint(const std::shared_ptr<Constraint>& c)
{
    mProxy->addConstraintSlot(c);
}

void SimulationControl::removeConstraint(const std::shared_ptr<Constraint>& c)
{
    mProxy->removeConstraintSlot(c);
}

void SimulationControl::addCollisionObject(
        std::shared_ptr<SimulationObject> so,
        double sphereDiameter)
{
    class CollisionObjectAdder : public SimulationObjectVisitor
    {
    public:
        CollisionObjectAdder(
                    SimulationControl& _sc,
                    double _sphereDiameter)
            : sc(_sc)
            , sphereDiameter(_sphereDiameter)
        {

        }

        virtual void visit(FEMObject& femObject)
        {
            // // discretize with spheres
            sc.mCollisionManagerProxy->addSimulationObject(
                        femObject.shared_from_this(),
                        femObject.getPolygon(),
                        sphereDiameter);

            // discretize with triangles
//            sc.mCollisionManagerProxy->addSimulationObjectTriangles(
//                        femObject.shared_from_this(),
//                        femObject.getPolygon());
        }

        virtual void visit(SimulationPoint& /*sp*/)
        {
            std::cout << "Can not craete CollisionObject from SimulationPoint\n";
        }

        virtual void visit(RigidBody& rigidBody)
        {
            // discretize with spheres
//            sc.mCollisionManagerProxy->addSimulationObject(
//                        rigidBody.shared_from_this(),
//                        rigidBody.getPolygon(),
//                        sphereDiameter);

            // discretize with triangles
            sc.mCollisionManagerProxy->addSimulationObjectTriangles(
                        rigidBody.shared_from_this(),
                        rigidBody.getPolygon());
        }

        SimulationControl& sc;
        double sphereDiameter;
    } v(*this, sphereDiameter);
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

void SimulationControl::initializeStep()
{
    mFEMSimulation->initializeStep();
    mRigidSimulation->initializeStep();

    // apply gravity
    for (std::shared_ptr<FEMObject>& femObj : mFEMSimulation->getFEMObjects())
    {
        for (size_t i = 0; i < femObj->getSize(); ++i)
        {
            femObj->applyForce(i, femObj->getMass(i) * mGravity);
        }
    }
    for (std::shared_ptr<RigidBody>& rb : mRigidSimulation->getRigidBodies())
    {
        rb->applyForce(Eigen::Vector::Zero(), rb->getMass() * mGravity);
    }

    // apply other forces
    applyForces();
}

void SimulationControl::step()
{
    if (mFEMSimulation->getFEMObjects().size() +
        mRigidSimulation->getRigidBodies().size() == 0)
    {
        return;
    }

    START_TIMING_SIMULATION("SimulationControl::step()");
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

    initializeStep();

    mRigidSimulation->solve(mStepSize);
    mFEMSimulation->solve(mStepSize, true); // x + x^{FEM} + x^{rigid}, v + v^{FEM} + v^{rigid}

    // Reverting the position before initializing the non-collision
    // constraints, so that only the position error from the previous
    // time step is corrected. Usually, the position error from this
    // predictor step is large and can make the simulation unstable.
    // Correcting the position error of the previous iteration is
    // equivalent to the baumgarte stabilization and deviates from
    // the approach of Benders "Constraint-based collision and contact handling
    // using impulses".
    mRigidSimulation->revertPositions();
    mFEMSimulation->revertPositions(); // x, v + v^{FEM} + v^{rigid}

    // initialize constraints
    mImpulseConstraintSolver->initializeNonCollisionConstraints(mStepSize);
//    mImpulseConstraintSolver->solveConstraints(30, 1e-5); // x, v + v^{nonh};

    mRigidSimulation->integratePositions(mStepSize);
    mFEMSimulation->integratePositions(mStepSize); // x + x^{FEM} + x^{rigid}, v + v^{FEM} + v^{rigid}

    mRigidSimulation->publish();
    mFEMSimulation->publish();

    START_TIMING_SIMULATION("CollisionManager::udpateAll()");
    mCollisionManager->updateAll();
    STOP_TIMING_SIMULATION;

    // collision detection on x + x^{FEM}
    START_TIMING_SIMULATION("CollisionManager::collideAll()");
    bool collisionsOccured = mCollisionManager->collideAll();
    STOP_TIMING_SIMULATION;

    if (collisionsOccured || !mImpulseConstraintSolver->getConstraints().empty())
    {
        // create collision constraints w.r.t. x + x^{FEM}
        mImpulseConstraintSolver->initializeCollisionConstraints(
                    mCollisionManager->getCollider()->getCollisions(),
                    mStepSize,
                    0.0, // Restitution (bounciness factor))
                    0.0, // static friction
                    0.001); // dynamic friction

        // Revert the illegal state
        mRigidSimulation->revertPositions();
        mFEMSimulation->revertPositions(); // x, v + v^{FEM}

        mImpulseConstraintSolver->solveConstraints(
                    mMaxNumConstraintSolverIterations,
                    mMaxConstraintError); // x, v + v^{FEM} + v^{col}

        if (!mFEMSimulation->getFEMObjects().empty())
        {
            for (int i = 0; i < mNumFEMCorrectionIterations; ++i)
            {
                mFEMSimulation->revertSolverStep(); // x, v + v^{col}

                // solve on x, v + v^{col}
//                START_TIMING_SIMULATION("CollisionManager::solve_FEM");
                mFEMSimulation->solve(mStepSize, false); // x + x^{FEM}, v + v^{FEM}
//                STOP_TIMING_SIMULATION;

//                mImpulseConstraintSolver->initializeCollisionConstraints(
//                            mCollisionManager->getCollider()->getCollisions(),
//                            mStepSize,
//                            0.0, // Restitution (bounciness factor))
//                            0.0, // static friction
//                            0.01); // dynamic friction

                mFEMSimulation->revertPositions(); // x, v + v^{FEM}

//                START_TIMING_SIMULATION("CollisionManager::solve_constraints");
                mImpulseConstraintSolver->solveConstraints(
                            mMaxNumConstraintSolverIterations,
                            mMaxConstraintError); // x, v + v^{FEM} + v^{col}
//                STOP_TIMING_SIMULATION;
            }
        }

        // x, v + v^{FEM} + v^{col}
        mRigidSimulation->integratePositions(mStepSize);
        mFEMSimulation->integratePositions(mStepSize); // x + x^{FEM} + x^{col}, v + v^{FEM} + v^{col}
    }


    mRigidSimulation->applyDamping();
    mFEMSimulation->applyDamping();

    START_TIMING_SIMULATION("CollisionManager::publish()");
    mRigidSimulation->publish();
    mFEMSimulation->publish();
    STOP_TIMING_SIMULATION;

    STOP_TIMING_SIMULATION;
}

void SimulationControl::setCollisionRenderingLevel(int level)
{
    mCollisionControl->setBvhRenderLevel(level);
}

void SimulationControl::setBVHCollisionRenderingEnabled(bool enabled)
{
    mCollisionControl->setBvhRenderingEnables(enabled);
}

bool SimulationControl::isCollisionNormalsVisible() const
{
    return mCollisionControl->isCollisionsRenderingVisible();
}

void SimulationControl::setCollisionNormalsVisible(bool visible)
{
    mCollisionControl->setCollisionsRenderingVisible(visible);
}

Domain* SimulationControl::getDomain()
{
    return mDomain;
}

void SimulationControl::addForceSlot(const std::shared_ptr<Force>& force)
{
    // Forces don't need to be added to the simulations
    // they can be applied independently?
    // why are the simulations even needed? maybe
    // renaming them makes sense?
    // FEMSolver
    // RigidSolver...

    auto it = std::find(mForces.begin(), mForces.end(), force);
    if (it == mForces.end())
    {
        mForces.push_back(force);
    }
}

void SimulationControl::removeForceSlot(const std::shared_ptr<Force>& force)
{
    auto it = std::find(mForces.begin(), mForces.end(), force);
    if (it != mForces.end())
    {
        mForces.erase(it);
    }
}

void SimulationControl::addConstraintSlot(const std::shared_ptr<Constraint>& c)
{
    auto it = std::find(mConstraints.begin(), mConstraints.end(), c);
    if (it == mConstraints.end())
    {
        mConstraints.push_back(c);
    }

    mImpulseConstraintSolver->addConstraint(c);

    for (SimulationControlListener* l : mListeners)
        l->onConstraintAdded(c);
}

void SimulationControl::removeConstraintSlot(const std::shared_ptr<Constraint>& c)
{
    auto it = std::find(mConstraints.begin(), mConstraints.end(), c);
    if (it != mConstraints.end())
    {
        mConstraints.erase(it);
    }

    mImpulseConstraintSolver->removeConstraint(c);

    for (SimulationControlListener* l : mListeners)
        l->onConstraintRemoved(c);
}

void SimulationControl::applyForces()
{
    for (const std::shared_ptr<Force>& f : mForces)
    {
        f->applyForce();
    }
}
