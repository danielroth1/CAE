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
#include <scene/data/geometric/MeshInterpolationManager.h>
#include <scene/data/geometric/MeshInterpolatorFEM.h>
#include <scene/data/geometric/AbstractPolygon.h>
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
    mMaxNumConstraintSolverIterations = 10;
    mMaxConstraintError = 1e-4;
    mPositionCorrectionFactor = 0.05;
    mContactMargin = 0.0015;

    mWarmStarting = true;
}

SimulationControl::~SimulationControl()
{
    delete mSimulationThread;
    delete mDomain;
}

std::shared_ptr<SimulationControlProxy> SimulationControl::getProxy() const
{
    return mProxy;
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

void SimulationControl::setPositionCorrectionFactor(double positionCorrectionFactor)
{
    mPositionCorrectionFactor = positionCorrectionFactor;
}

double SimulationControl::getPositionCorrectionFactor() const
{
    return mPositionCorrectionFactor;
}

void SimulationControl::setCollisionMargin(double collisionMargin)
{
    mCollisionManager->setCollisionMargin(collisionMargin);
}

double SimulationControl::getCollisionMargin() const
{
    return mCollisionManager->getCollisionMargin();
}

void SimulationControl::setContactMargin(double contactMargin)
{
    mContactMargin = contactMargin;
}

double SimulationControl::getContactMargin() const
{
    return mContactMargin;
}

void SimulationControl::setWarmStarting(bool warmStarting)
{
    mWarmStarting = warmStarting;
}

bool SimulationControl::isWarmStaring() const
{
    return mWarmStarting;
}

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

    mSimulationObjects.erase(it);
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
        const std::shared_ptr<SimulationObject>& so,
        const std::shared_ptr<MeshInterpolatorFEM>& interpolation,
        double sphereDiameter)
{
    class CollisionObjectAdder : public SimulationObjectVisitor
    {
    public:
        CollisionObjectAdder(
                    SimulationControl& _sc,
                    const std::shared_ptr<MeshInterpolatorFEM>& _interpolation,
                    double _sphereDiameter)
            : sc(_sc)
            , interpolation(_interpolation)
            , sphereDiameter(_sphereDiameter)
        {

        }

        virtual void visit(FEMObject& femObject)
        {
            // Mass-pont based collisions handling -> a colliison sphere put
            // to where each mass point is.
//            sc.mCollisionManagerProxy->addSimulationObject(
//                        femObject.shared_from_this(),
//                        femObject.getPolygon(),
//                        sphereDiameter);

            if (interpolation)
            {
                sc.mCollisionManagerProxy->addSimulationObjectTriangles(
                            femObject.shared_from_this(),
                            interpolation);
            }
            else
            {
                // Triangle based collision handling
                sc.mCollisionManagerProxy->addSimulationObjectTriangles(
                            femObject.shared_from_this(),
                            femObject.getPolygon());
            }
        }

        virtual void visit(SimulationPoint& /*sp*/)
        {
            std::cout << "Can not craete CollisionObject from SimulationPoint\n";
        }

        virtual void visit(RigidBody& rigidBody)
        {
            // Use spheres to dicetrize each triangle of the body (not only
            // the vertices like in the FEM case).
//            sc.mCollisionManagerProxy->addSimulationObject(
//                        rigidBody.shared_from_this(),
//                        rigidBody.getPolygon(),
//                        sphereDiameter);

            if (interpolation)
            {
                sc.mCollisionManagerProxy->addSimulationObjectTriangles(
                            rigidBody.shared_from_this(),
                            interpolation);
            }
            else
            {
                // discretize with triangles
                sc.mCollisionManagerProxy->addSimulationObjectTriangles(
                            rigidBody.shared_from_this(),
                            rigidBody.getPolygon());
            }
        }

        SimulationControl& sc;
        const std::shared_ptr<MeshInterpolatorFEM>& interpolation;
        double sphereDiameter;
    } v(*this, interpolation, sphereDiameter);
    so->accept(v);
}

void SimulationControl::addCollisionObject(
        const std::shared_ptr<MeshInterpolatorFEM>& interpolation)
{
    auto it = std::find_if(
                mSimulationObjects.begin(), mSimulationObjects.end(),
                [interpolation](const std::shared_ptr<SimulationObject>& so)
    {
        if (so->getType() == SimulationObject::Type::FEM_OBJECT)
        {
            return std::static_pointer_cast<FEMObject>(so)->getPolygon() == interpolation->getSource();
        }
        else if (so->getType() == SimulationObject::Type::RIGID_BODY)
        {
            return std::static_pointer_cast<RigidBody>(so)->getPolygon() == interpolation->getSource();
        }
        return false;
    });
    if (it != mSimulationObjects.end())
    {
        addCollisionObject(*it, interpolation);
    }
}

void SimulationControl::addCollisionObject(const std::shared_ptr<AbstractPolygon>& poly)
{
    // Check if the polygon is owned by a SimulationObject and if yes, set
    // that as collidable.
    auto it = std::find_if(
                mSimulationObjects.begin(), mSimulationObjects.end(),
                [poly](const std::shared_ptr<SimulationObject>& so)
    {
        if (so->getType() == SimulationObject::Type::FEM_OBJECT)
        {
            return std::static_pointer_cast<FEMObject>(so)->getPolygon() == poly;
        }
        else if (so->getType() == SimulationObject::Type::RIGID_BODY)
        {
            return std::static_pointer_cast<RigidBody>(so)->getPolygon() == poly;
        }
        return false;
    });
    if (it != mSimulationObjects.end())
    {
        addCollisionObject(*it, nullptr);
    }
    else
    {
        // Check if it's referenced by an interpolator and if yes, use that
        // one.
        std::shared_ptr<MeshInterpolator> interpolator =
                mAc->getMeshInterpolationManager()->getInterpolator(poly);
        if (interpolator &&
                interpolator->getType() == MeshInterpolator::Type::FEM)
        {

            auto it = std::find_if(
                        mSimulationObjects.begin(), mSimulationObjects.end(),
                        [interpolator](const std::shared_ptr<SimulationObject>& so)
            {
                if (so->getType() == SimulationObject::Type::FEM_OBJECT)
                {
                    return std::static_pointer_cast<FEMObject>(so)->getPolygon() == interpolator->getSource();
                }
                else if (so->getType() == SimulationObject::Type::RIGID_BODY)
                {
                    return std::static_pointer_cast<RigidBody>(so)->getPolygon() == interpolator->getSource();
                }
                return false;
            });
            if (it != mSimulationObjects.end())
            {
                addCollisionObject(*it, std::static_pointer_cast<MeshInterpolatorFEM>(interpolator));
                return;
            }
        }
    }
}

void SimulationControl::removeCollisionObject(const std::shared_ptr<SimulationObject>& so)
{
    mCollisionManagerProxy->removeSimulationObject(so);
}

void SimulationControl::removeCollisionObject(const std::shared_ptr<AbstractPolygon>& poly)
{
    mCollisionManager->removePolygon(poly);
}

void SimulationControl::addCollisionGroup(
        const std::shared_ptr<SimulationObject>& so, int groupId)
{
    mCollisionManagerProxy->addCollisionGroupId(so, groupId);
}

void SimulationControl::setCollisionGroups(
        const std::shared_ptr<SimulationObject>& so,
        const std::vector<int>& collisionGroupIds)
{
    mCollisionManagerProxy->setCollisionGroupIds(so, collisionGroupIds);
}

bool SimulationControl::isCollidable(const std::shared_ptr<AbstractPolygon>& poly)
{
    if (poly != nullptr)
    {
        return mCollisionManager->isCollidable(poly);
    }
    return false;
}

bool SimulationControl::isCollidable(
        const std::shared_ptr<SimulationObject>& so)
{
    if (so != nullptr)
    {
        return mCollisionManager->isCollidable(
                    std::dynamic_pointer_cast<AbstractPolygon>(
                        dynamic_cast<AbstractPolygon*>(
                            so->getGeometricData())->shared_from_this()));
    }
    return false;
}

void SimulationControl::setCollidable(
        const std::shared_ptr<SimulationObject>& so, bool collidable)
{
    if (collidable)
        addCollisionObject(so, nullptr);
    else
        removeCollisionObject(so);
}

void SimulationControl::setCollidable(
        const std::shared_ptr<MeshInterpolatorFEM>& interpolator, bool collidable)
{
    if (collidable)
        addCollisionObject(interpolator);
    else
        removeCollisionObject(interpolator->getTarget());
}

void SimulationControl::setCollidable(const std::shared_ptr<AbstractPolygon>& poly, bool collidable)
{
    if (collidable)
        addCollisionObject(poly);
    else
        removeCollisionObject(poly);
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

    START_TIMING_SIMULATION("SimulationControl::step::initializeStep");
    initializeStep();
    STOP_TIMING_SIMULATION;

    // Perform some kind of early collision detection before the predictor
    // step to obtain a better result.

    // collision search gives collisions
    // creating collision constraints from those
    // performing predictor step
    // contact search gives new contacts -> also onces that are identical to already found ones
    // create collision constraints from those and the previously found ones

    // reuse old collisions, but first check if they are still valid
    // all non-valid collisions are removed
    START_TIMING_SIMULATION("SimulationControl::step::revalidateCollisions");
    mCollisionManager->revalidateCollisions();
    size_t numPrevCollisions = mCollisionManager->getCollisions().size();
    STOP_TIMING_SIMULATION;

    START_TIMING_SIMULATION("SimulationControl::step::revalidateCollisionConstraints");
    mImpulseConstraintSolver->revalidateCollisionConstraints(
                mCollisionManager->getCollisions(),
                mStepSize,
                0.0, // Restitution (bounciness factor))
                mPositionCorrectionFactor,
                mCollisionManager->getCollisionMargin(),
                mContactMargin,
                true,
                mWarmStarting);
    STOP_TIMING_SIMULATION;

    START_TIMING_SIMULATION("SimulationControl::step::solveVelocity");
    mRigidSimulation->solveVelocity(mStepSize);
    mFEMSimulation->solveVelocity(mStepSize, true); // x + x^{FEM} + x^{rigid}, v + v^{FEM} + v^{rigid}
    STOP_TIMING_SIMULATION;

    // initialize constraints
    mImpulseConstraintSolver->initializeNonCollisionConstraints(mStepSize);

    // calculation of predictor step + collision constraint creation

    // warm starting
    if (mWarmStarting && !mCollisionManager->getCollisions().empty())
    {
        mImpulseConstraintSolver->applyWarmStarting();
    }

    // solve initial contacts (if there are any) and joints
    if (!mCollisionManager->getCollisions().empty() ||
            !mImpulseConstraintSolver->getConstraints().empty())
    {
        mImpulseConstraintSolver->solveConstraints(
                    mMaxNumConstraintSolverIterations, mMaxConstraintError);
    }

    mRigidSimulation->integratePositions(mStepSize);
    mFEMSimulation->integratePositions(mStepSize);

    START_TIMING_SIMULATION("SimulationControl::step::initialPublish");
//    mRigidSimulation->publish(false);
    mFEMSimulation->publish(false);
    mCollisionManager->updateGeometries();
    STOP_TIMING_SIMULATION;

    START_TIMING_SIMULATION("SimulationControl::step::udpateAll");
    mCollisionManager->updateAll();
    STOP_TIMING_SIMULATION;

    // collision detection on x + x^{FEM}
    START_TIMING_SIMULATION("SimulationControl::step::collideAll");
    bool predictorCollisionsOccured = mCollisionManager->collideAll();
    STOP_TIMING_SIMULATION;

    mRigidSimulation->revertPositions();
    mFEMSimulation->revertPositions(); // x, v + v^{FEM}

    if (predictorCollisionsOccured)
    {
        // no need to reinitialize the previous collision constraints
        // -> can save some time here.
        mImpulseConstraintSolver->initializeCollisionConstraints(
                    mCollisionManager->getCollisions(),
                    numPrevCollisions,
                    mStepSize,
                    0.0, // Restitution (bounciness factor))
                    mPositionCorrectionFactor,
                    mCollisionManager->getCollisionMargin(),
                    mContactMargin,
                    false);
    }

    START_TIMING_SIMULATION("SimulationControl::step::mixed_loop");
    if (!mCollisionManager->getCollisions().empty() ||
            !mImpulseConstraintSolver->getConstraints().empty())
    {
        mImpulseConstraintSolver->solveConstraints(
                    mMaxNumConstraintSolverIterations,
                    mMaxConstraintError); // x, v + v^{FEM} + v^{col}

        if (!mFEMSimulation->getFEMObjects().empty())
        {
            for (int i = 0; i < mNumFEMCorrectionIterations; ++i)
            {
                mFEMSimulation->revertSolverStep(); // x, v + v^{col}

                // solve on x, v + v^{col}
                mFEMSimulation->solveVelocity(mStepSize, false); // x + x^{FEM}, v + v^{FEM}

                mImpulseConstraintSolver->solveConstraints(
                            mMaxNumConstraintSolverIterations,
                            mMaxConstraintError); // x, v + v^{FEM} + v^{col}
            }
        }

    }
    // x, v + v^{FEM} + v^{col}
    mRigidSimulation->integratePositions(mStepSize);
    mFEMSimulation->integratePositions(mStepSize); // x + x^{FEM} + x^{col}, v + v^{FEM} + v^{col}
    STOP_TIMING_SIMULATION;

    mRigidSimulation->applyDamping();
    mFEMSimulation->applyDamping();

    START_TIMING_SIMULATION("SimulationControl::step::publish");
    mRigidSimulation->publish();
    mFEMSimulation->publish();
    STOP_TIMING_SIMULATION;

    START_TIMING_SIMULATION("SimulationControl::step::udpateSimulationObjects");
    mAc->updateSimulationObjects();
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
