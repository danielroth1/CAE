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

#include <QObject>

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
class RigidBody;
class ImpulseConstraintSolver;
class RigidSimulation;
class RigidSimulationProxy;
class SimulationControlListener;
class SimulationThread;
class StepperThread;
class UIControl;

// Manages the currently running simulation.
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
class SimulationControl : public QObject
{
public:

    SimulationControl();

    virtual ~SimulationControl();

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
    // LinearForceRenderModel.
    void addLinearForce(std::shared_ptr<LinearForce> linearForce);

    // Adds the given linear force to the simulation and creates a
    // LinearForceRenderModel.
    void addLinearForce(
            SimulationObject* source, ID sourceVector,
            SimulationObject* target, ID targetVector,
            double strength);

    // Inits the FEM simulation data.
    // Loads a mesh.
    // Starts the simulation thread.
    void initialize();
    void startSimulationThread();

    void setGravity(Eigen::Vector gravity);
    Eigen::Vector getGravity() const;

    void setSimulationPaused(bool paused);
    bool isSimulationPaused();

    void setStepSize(double stepSize);
    double getStepSize() const;

    void setNumFEMCorrectionIterations(int correctionIterations);
    int getNumFEMCorrectionIterations() const;

    // repaints the gl widget
//    void repaint();

    // Simulation access methods
        std::shared_ptr<FEMSimulation> getFEMSimulation();
        void addSimulationObject(std::shared_ptr<SimulationObject> so);

        // Removes the given simulation object from the simulation.
        // Also removes all constraints/ potentials that are assosiated with that
        // SimulationObject.
        void removeSimulationObject(const std::shared_ptr<SimulationObject>& so);

        // Removes all simulation objects from the simulation.
        void clearSimulationObjects();

        void addForce(const std::shared_ptr<Force>& f);
        void removeForce(const std::shared_ptr<Force>& f);

        void addConstraint(const std::shared_ptr<Constraint>& c);
        void removeConstraint(const std::shared_ptr<Constraint>& c);

        void addCollisionObject(std::shared_ptr<SimulationObject> so);
        void removeCollisionObject(const std::shared_ptr<SimulationObject>& so);

    // Is called by SimulationThread once after the thread is started.
    void initializeSimulation();

    void initializeStep();

    // Is called by SimulationThread a certain amount of time per second.
    void step();

    std::vector<std::shared_ptr<Simulation>>& getSimulations();

    // Rendering
        void setCollisionRenderingLevel(int level);
        void setBVHCollisionRenderingEnabled(bool enabled);

    //void simulationLoop();

    Domain* getDomain();

    // Delegated Simulations
public:
    void addSimulation(const std::shared_ptr<Simulation>& simulation,
                       const std::shared_ptr<SimulationProxy>& proxy);

public slots:

//signals:
//    void repaintSignal();


private:

    // Handles what happens after a simulation step
    void handleAfterStep();

    void applyForces();

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

    int mNumFEMCorrectionIterations;

    bool mPaused;

//    GLWidget* m_gl_widget;

//    MainWindow* m_main_window;



    // use this to enable the usage of shared OpenGL context
    // using this limits the context to only store data (buffer)
    // and can, therefore, not be used to draw
//    QOffscreenSurface m_surface;


};

//PROXY_CLASS(SimulationControlProxy, SimulationControl, mSc,
//            PROXY_FUNCTION(SimulationControl, mSc, actForce,
//                           PL(Selection* selection, Eigen::Vector force),
//                           PL(selection, force))
//            PROXY_FUNCTION(SimulationControl, mSc, clearTruncations, , )
//            PROXY_FUNCTION(SimulationControl, mSc, addTruncations,
//                           PL(FEMObject* femObject, std::vector<ID> vectorIDs),
//                           PL(femObject, vectorIDs))
//            PROXY_FUNCTION(SimulationControl, mSc, initializeSimulation, , )
//            PROXY_FUNCTION(SimulationControl, mSc, addSimulationObject,
//                           PL(std::shared_ptr<SimulationObject> so),
//                           PL(so))
//            PROXY_FUNCTION(SimulationControl, mSc, removeSimulationObject,
//                           PL(SimulationObject* so),
//                           PL(so))
//            )


//PROXY_CLASS(SimulationControlProxy, SimulationControl, mSc,
//            PROXY_FUNCTION(SimulationControl, mSc, initialize)
//            PROXY_FUNCTION(SimulationControl, mSc, actForce, Selection* s, Eigen::Vector v))

#endif // SIMULATIONCONTROL_H
