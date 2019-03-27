#ifndef RIGIDSIMULATION_H
#define RIGIDSIMULATION_H

#include <simulation/Simulation.h>

class Domain;
class RigidBody;
class RigidCollisionSolver;

class RigidSimulation : public Simulation
{
public:
    RigidSimulation(
            Domain* domain,
            double timeStep,
            std::shared_ptr<CollisionManager> collisionManager);

    virtual ~RigidSimulation() override;

    // Simulation objects contain information on the
    // the size of the required space in the global state.
    // The space is allocated and the global id to the
    // allocated space is assigned. It is up to the
    // simulation object to use the assigned space.
    // A reorganization of the global state can happen,
    // but not while a method call in the SimulationObject.
    void addRigidBody(std::shared_ptr<RigidBody> rigidBody);

    bool removeRigidBody(RigidBody* rigidBody);

    // Call this before calling solve.
    void initializeStep();

    // Calculates forces and updates the velocities according to physics.
    void solve();

    // Time integrates the velocity to update the positions.
    // The previous position state can be reverted to by calling revert().
    void integratePositions();

    // Reverts the positions to the state before the previous integratePositions()
    // call.
    void revertPositions();

    void applyDamping();

    // Tells the renderer, that the data changed and can be visualized.
    // Call this after a position change should be visualized.
    void publish();

    // TODO:
    // SimulationObject::getSize()
    // SimulationObject::assignState(ID pos)
    // SimulationObject::assignDState(double* stateID pos)

    // Simulation interface
public:
    virtual void initialize() override;
    virtual void step() override;

    virtual void actExternalForce(
            SimulationObject* so,
            ID vertexIndex,
            Eigen::Vector force) override;

    virtual void actExternalForce(
            SimulationObject* so,
            Eigen::Vector r,
            Eigen::Vector force) override;

    // Calculates and returns the relative speed of two
    // contact points in normal direction.
    static Eigen::Vector calculateRelativeSpeed(
            RigidBody* body1, const Eigen::Vector& r1,
            RigidBody* body2, const Eigen::Vector& r2,
            const Eigen::Vector& normal);

    // Calculates and returns a collision correction impulse
    // 1 / (n^T K n) * Delta_u_rel.
    static Eigen::Vector calculateCollisionCorrectionImpulse(
            RigidBody* body1, const Eigen::Vector& r1,
            RigidBody* body2, const Eigen::Vector& r2,
            const Eigen::Vector& targetURel,
            const Eigen::Vector& normal);

private:

    void solveExplicitly();
    void applyLinearToExternalForces();

//    for(int i = 0; i < nCollisionIterations; ++i)
//        applyCollisionCorrectionImpulses()
//        apply constraint impulses
//        advance time step
//        if (collideAll())
//            revertTimeStep()

    // Stores position state
    // Simulation objects should remember an id that points
    // to the starting position of their position data
    double* mState;

    // Stores velocity state
    // Same as for position state.
    double* mDState;

    std::vector<std::shared_ptr<RigidBody>> mRigidBodies;
};

PROXY_CLASS_DERIVED(RigidSimulationProxy, RigidSimulation, SimulationProxy, mRigidSimulation,

            PROXY_FUNCTION(RigidSimulation, mRigidSimulation, addRigidBody,
                           PL(std::shared_ptr<RigidBody> rigidBody),
                           PL(rigidBody))

            PROXY_FUNCTION(RigidSimulation, mRigidSimulation, removeRigidBody,
                           PL(RigidBody* rigidBody),
                           PL(rigidBody))

            )

#endif // RIGIDSIMULATION_H
