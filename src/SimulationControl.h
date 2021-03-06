#ifndef SIMULATIONCONTROL_H
#define SIMULATIONCONTROL_H


//#include <QOffscreenSurface>

#include <simulation/Simulation.h>
#include <data_structures/DataStructures.h>
#include <ui/selection/Selection.h>
//#include <main_window.h>

//#include <QOpenGLContext>

#include <thread>
#include <atomic>
#include <memory> // shared_ptr

class ApplicationControl;
class CollisionControl;
class CollisionManager;
class CollisionManagerProxy;
class Constraint;
class Domain;
class Force;
class GLWidget;
//class Selection;
class FEMSimulation;
class FEMSimulationProxy;
class FEMObject;
class ImpulseConstraintSolver;
class MeshInterpolatorFEM;
class RigidBody;
class RigidSimulation;
class RigidSimulationProxy;
class SimulationControlListener;
class SimulationControlProxy;
class SimulationThread;
class StepperThread;
class UIControl;

// Manages the currently running simulation.
// Implements the simulation loop, see step().
//
// This class acts as a wrapper for simulation instances. It provides
// no functionality to change the simulations directly. Simulations
// are accessed by retrieving the simulation vector and changing
// the them directly.
//
// Actions:
// Also implements actions that can be called by everyone.
// The reason for that is the seperation of UI and logic.
// UIControl calles the actions anytime a user interaction
// is registered.
//
class SimulationControl : public std::enable_shared_from_this<SimulationControl>
{
public:

    SimulationControl();

    virtual ~SimulationControl();

    std::shared_ptr<SimulationControlProxy> getProxy() const;

    void setApplicationControl(ApplicationControl* ac);

    void connectSignals(GLWidget& glWidget);

    void handleFileInput(int argc, char *argv[]);

    bool addListener(SimulationControlListener* listener);
    bool removeListener(SimulationControlListener* listener);

    // Act force
    // TODO: visualize vertices on which a force acts
    void actForce(Selection* selection, Eigen::Vector force);

    // Truncations
    // TODO: visualize vertices that are truncated
    void clearTruncations();
    void addTruncations(FEMObject* femObject, std::vector<ID> vectorIDs);

    // Adds the given linear force to the simulation and creates a
    // LinearForceRenderModel to render it.
    void addLinearForce(std::shared_ptr<LinearForce> linearForce);

    // Adds the given linear force to the simulation and creates a
    // LinearForceRenderModel to render it.
    void addLinearForce(
            SimulationObject* source, ID sourceVector,
            SimulationObject* target, ID targetVector,
            double strength);

    // Inits the FEM simulation data.
    // Loads a mesh.
    // Starts the simulation thread.
    void initialize();
    void startSimulationThread();

    void performSingleStep();

    void setGravity(Eigen::Vector gravity);
    Eigen::Vector getGravity() const;

    void setSimulationPaused(bool paused);
    bool isSimulationPaused();

    void setStepSize(double stepSize);
    double getStepSize() const;

    void setMaxNumConstraintSolverIterations(int numConstraintSolverIterations);
    int getMaxNumConstraintSolverIterations() const;

    void setMaxConstraintError(double maxConstraintError);
    double getMaxConstraintError() const;

    void setNumFEMCorrectionIterations(int correctionIterations);
    int getNumFEMCorrectionIterations() const;

    void setInvertNormalsIfNecessary(bool invertNormalsIfNecessary);
    bool getInvertNormalsIfNecessary() const;

    void setPositionCorrectionFactor(double positionCorrectionFactor);
    double getPositionCorrectionFactor() const;

    void setCollisionMargin(double collisionMargin);
    double getCollisionMargin() const;

    void setContactMargin(double contactMargin);
    double getContactMargin() const;

    void setWarmStarting(bool warmStarting);
    bool isWarmStaring() const;

    // Simulation access methods
        std::shared_ptr<FEMSimulation> getFEMSimulation();
        // Thread safe
        void addSimulationObject(
                const std::shared_ptr<SimulationObject>& so);

        // Thread safe
        // Removes the given simulation object from the simulation.
        // Also removes all constraints/ potentials that are assosiated with that
        // SimulationObject.
        void removeSimulationObject(const std::shared_ptr<SimulationObject>& so);

        // Thread safe
        // Removes all simulation objects from the simulation.
        void clearSimulationObjects();

        // Thread safe
        void addForce(const std::shared_ptr<Force>& f);
        // Thread safe
        void removeForce(const std::shared_ptr<Force>& f);

        void addConstraint(const std::shared_ptr<Constraint>& c);
        void removeConstraint(const std::shared_ptr<Constraint>& c);

        // Makes the given SimulationObject collidable. Does nothing if it's
        // already collidable.
        // If interpolation != nullptr, the target polygon of the interpolator
        // is used as the collision geometry. In that case the polygon of so
        // must be the same as the source polygon of the interpolator.
        void addCollisionObject(
                const std::shared_ptr<SimulationObject>& so,
                const std::shared_ptr<MeshInterpolatorFEM>& interpolation,
                double collisionSphereRadiusFactor = 0.2);

        // Adds the target polygon of the given interpolator as collision
        // geometry. The source polygon must be assigned to simulation object
        // and added to the simulation,
        // i.e. SimulationControl::addSimulationObject
        // else the method does nothing.
        void addCollisionObject(
                const std::shared_ptr<MeshInterpolatorFEM>& interpolation);

        // Makes the given polygon collidable.
        // One of the following conditions must be true:
        // - the polygon is part of a simulation object that was added to the
        //   simulation, i.e. SimulationControl::addSimulationObject()
        // - the polygon is the target of an FEMInterpolator and the source
        //   polygon of that interpolator is part of a simulation object that
        //   was added to the simulation.
        void addCollisionObject(const std::shared_ptr<AbstractPolygon>& poly);

        // Makes the given simulation object non-collidable. If it's already
        // nothing happens. Also works if the target polygon of an interpolator
        // is collidable.
        void removeCollisionObject(const std::shared_ptr<SimulationObject>& so);

        // Removes the given polygon from the collision detection. It's either
        // directly a simulation object or the target of an interpolator.
        void removeCollisionObject(const std::shared_ptr<AbstractPolygon>& poly);

        void addCollisionGroup(
                const std::shared_ptr<SimulationObject>& so, int groupId);

        void setCollisionGroups(
                const std::shared_ptr<SimulationObject>& so,
                const std::vector<int>& collisionGroupIds);

        bool isCollidable(const std::shared_ptr<AbstractPolygon>& poly);
        bool isCollidable(const std::shared_ptr<SimulationObject>& so);

        // Convenience method that either calls addCollisionObject() or
        // removeCollisionObject().
        void setCollidable(const std::shared_ptr<SimulationObject>& so, bool collidable);

        // Convenience method that either calls addCollisionObject() or
        // removeCollisionObject().
        void setCollidable(const std::shared_ptr<MeshInterpolatorFEM>& interpolator, bool collidable);

        // Convenience method that either calls addCollisionObject() or
        // removeCollisionObject().
        void setCollidable(const std::shared_ptr<AbstractPolygon>& poly, bool collidable);


    // Is called by SimulationThread once after the thread is started.
    void initializeSimulation();

    void initializeStep();

    // Is called by SimulationThread a certain amount of time per second.
    // Collisions are generated by performing a predictor step wihtout collision
    // constraints. Only updates collidable geometries after the predictor step.
    //
    // - collisions from previous time steps are kept if they are still valid
    //   (see CollisionManager::revalidateCollisions())
    // - reused collisions are used for warm starting the corresponsing
    //   collision constraints
    //   (see ImpulseConstraintSolver::revalidateCollisionConstraints() and
    //        ImpulseConstraintSolver::applyWarmStarting())
    void step();

    std::vector<std::shared_ptr<Simulation>>& getSimulations();

    // Rendering
        void setCollisionRenderingLevel(int level);
        void setBVHCollisionRenderingEnabled(bool enabled);

        bool isCollisionNormalsVisible() const;
        void setCollisionNormalsVisible(bool visible);

    //void simulationLoop();

    Domain* getDomain();

    // Delegated SimulationspositionCorrectionFactor
public:
    void addSimulation(const std::shared_ptr<Simulation>& simulation,
                       const std::shared_ptr<SimulationProxy>& proxy);

    // slots
public:
    void addForceSlot(const std::shared_ptr<Force>& force);
    void removeForceSlot(const std::shared_ptr<Force>& force);
    void addConstraintSlot(const std::shared_ptr<Constraint>& c);
    void removeConstraintSlot(const std::shared_ptr<Constraint>& c);

private:

    void applyForces();

    // Retrieves all constraints that reference the given simulation object
    // and returns them in a vector.
    template <class MechanicalPropertyType>
    std::vector<std::shared_ptr<MechanicalPropertyType>>
    retrieveReferencingMechanicalProperties(
            const std::shared_ptr<SimulationObject>& so,
            const std::vector<std::shared_ptr<MechanicalPropertyType>>& constraints)
    {
        std::vector<std::shared_ptr<MechanicalPropertyType>> conRef;
        for (const std::shared_ptr<MechanicalPropertyType>& c : constraints)
        {
            if (c->references(so.get()))
            {
                conRef.push_back(c);
            }
        }
        return conRef;
    }

    std::shared_ptr<SimulationControlProxy> mProxy;

    ApplicationControl* mAc;
    UIControl* mUiControl;

    std::vector<SimulationControlListener*> mListeners;
    std::vector<std::shared_ptr<SimulationObject>> mSimulationObjects;

    std::shared_ptr<FEMSimulation> mFEMSimulation;
    std::shared_ptr<FEMSimulationProxy> mFEMSimulationProxy;

    std::shared_ptr<RigidSimulation> mRigidSimulation;
    std::shared_ptr<RigidSimulationProxy> mRigidSimulationProxy;

    unsigned int mUpdateInterval;

    SimulationThread* mSimulationThread;

    Domain* mDomain;

    std::vector<std::shared_ptr<Force>> mForces;
    std::vector<std::shared_ptr<Constraint>> mConstraints;

    std::shared_ptr<CollisionControl> mCollisionControl;
    std::unique_ptr<ImpulseConstraintSolver> mImpulseConstraintSolver;

    // Is owned by CollisionControl, SimulationControl, and
    // all Simulations
    std::shared_ptr<CollisionManager> mCollisionManager;
    std::shared_ptr<CollisionManagerProxy> mCollisionManagerProxy;

    Eigen::Vector mGravity;

    double mStepSize;

    int mMaxNumConstraintSolverIterations;
    int mNumFEMCorrectionIterations;
    double mMaxConstraintError;
    double mPositionCorrectionFactor;
    double mContactMargin;
    bool mWarmStarting;

    bool mPaused;

//    GLWidget* m_gl_widget;

//    MainWindow* m_main_window;



    // use this to enable the usage of shared OpenGL context
    // using this limits the context to only store data (buffer)
    // and can, therefore, not be used to draw
//    QOffscreenSurface m_surface;


};

PROXY_CLASS(SimulationControlProxy, SimulationControl, mSc,
            PROXY_FUNCTION(SimulationControl, mSc, step,
                           PL(),
                           PL())
            PROXY_FUNCTION(SimulationControl, mSc, removeSimulationObject,
                           PL(const std::shared_ptr<SimulationObject>& so),
                           PL(so))
            PROXY_FUNCTION(SimulationControl, mSc, addForceSlot,
                           PL(const std::shared_ptr<Force>& force),
                           PL(force))
            PROXY_FUNCTION(SimulationControl, mSc, removeForceSlot,
                           PL(const std::shared_ptr<Force>& force),
                           PL(force))
            PROXY_FUNCTION(SimulationControl, mSc, addConstraintSlot,
                           PL(const std::shared_ptr<Constraint>& constraint),
                           PL(constraint))
            PROXY_FUNCTION(SimulationControl, mSc, removeConstraintSlot,
                           PL(const std::shared_ptr<Constraint>& constraint),
                           PL(constraint))
            PROXY_FUNCTION(SimulationControl, mSc, addCollisionObject,
                           PL(const std::shared_ptr<SimulationObject>& so,
                              const std::shared_ptr<MeshInterpolatorFEM>& interpolation,
                              double collisionSphereRadiusFactor),
                           PL(so, interpolation, collisionSphereRadiusFactor))
            PROXY_FUNCTION(SimulationControl, mSc, addCollisionObject,
                           PL(const std::shared_ptr<MeshInterpolatorFEM>& interpolation),
                           PL(interpolation))
            PROXY_FUNCTION(SimulationControl, mSc, addCollisionObject,
                           PL(const std::shared_ptr<AbstractPolygon>& poly),
                           PL(poly))
            PROXY_FUNCTION(SimulationControl, mSc, removeCollisionObject,
                           PL(const std::shared_ptr<SimulationObject>& so),
                           PL(so))
            PROXY_FUNCTION(SimulationControl, mSc, removeCollisionObject,
                           PL(const std::shared_ptr<AbstractPolygon>& poly),
                           PL(poly))
            PROXY_FUNCTION(SimulationControl, mSc, addCollisionGroup,
                           PL(const std::shared_ptr<SimulationObject>& so, int groupId),
                           PL(so, groupId))
            PROXY_FUNCTION(SimulationControl, mSc, setCollisionGroups,
                           PL(const std::shared_ptr<SimulationObject>& so,
                              const std::vector<int>& collisionGroupIds),
                           PL(so, collisionGroupIds))
            PROXY_FUNCTION(SimulationControl, mSc, setCollidable,
                           PL(const std::shared_ptr<SimulationObject>& so, bool collidable),
                           PL(so, collidable))
            PROXY_FUNCTION(SimulationControl, mSc, setCollidable,
                           PL(const std::shared_ptr<MeshInterpolatorFEM>& interpolator, bool collidable),
                           PL(interpolator, collidable))
            PROXY_FUNCTION(SimulationControl, mSc, setCollidable,
                           PL(const std::shared_ptr<AbstractPolygon>& poly, bool collidable),
                           PL(poly, collidable))
            )

#endif // SIMULATIONCONTROL_H
