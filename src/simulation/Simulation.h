#ifndef SIMULATION_H
#define SIMULATION_H

#include "data_structures/DataStructures.h"
#include "proxy/ProxyDefs.h"
#include "ui/UniqueVertex.h"
#include <memory> // shared_ptr
#include <simulation/references/SimulationPointRef.h>

class CollisionManager;
class Domain;
class LinearForce;
class SimulationPointRef;

// call these methods
// initialize();
// while (...) {
//   updateForcesAndStiffnessMatrix();
//   solve();
//   update();
// }
class Simulation : public std::enable_shared_from_this<Simulation>
{
public:
    Simulation(Domain* domain,
               double timeStep,
               std::shared_ptr<CollisionManager> collisionManager);

    virtual ~Simulation();

    virtual void initialize() = 0;
    virtual void step() = 0;

    // Act an external force on a simulation point.
    // This method dispatches to the other actExternalForce() methods, depending
    // on if the force is acted w.r.t. an arbitrary vector or w.r.t. a vertex with
    // index.
    void actExternalForce(const SimulationPointRef& ref, Eigen::Vector force);

    void addLinearForce(std::shared_ptr<LinearForce> linearForce);
    bool removeLinearForce(LinearForce* linearForce);
    std::vector<std::shared_ptr<LinearForce>>& getLinearForces();

    Domain* getDomain();
    double getTimeStep() const;


protected:
    virtual void actExternalForce(
            SimulationObject* so,
            ID vertexIndex,
            Eigen::Vector force) = 0;

    virtual void actExternalForce(
            SimulationObject* so,
            Eigen::Vector r,
            Eigen::Vector force) = 0;

    Domain* mDomain;

    double mTimeStep;

    bool mRunning;

    std::vector<std::shared_ptr<LinearForce>> mLinearForces;

    std::shared_ptr<CollisionManager> mCollisionManager;

};

PROXY_CLASS(SimulationProxy, Simulation, mS,
            PROXY_FUNCTION(Simulation, mS, initialize, , )
            PROXY_FUNCTION(Simulation, mS, step, , )
//            PROXY_FUNCTION(Simulation, mS, preSimulationStep, , )
//            PROXY_FUNCTION(Simulation, mS, solveExplicitly, , )
//            PROXY_FUNCTION(Simulation, mS, solve, , )
//            PROXY_FUNCTION(Simulation, mS, update, , )
//            PROXY_FUNCTION(Simulation, mS, applyLinearToExternalForces, , )

            PROXY_FUNCTION(Simulation, mS, addLinearForce,
                           PL(std::shared_ptr<LinearForce> linearForce),
                           PL(linearForce))

            PROXY_FUNCTION(Simulation, mS, removeLinearForce,
                           PL(LinearForce* linearForce),
                           PL(linearForce))

            PROXY_FUNCTION(Simulation, mS, actExternalForce,
                           PL(const SimulationPointRef& ref, Eigen::Vector force),
                           PL(ref, force))
            )

#endif // SIMULATION_H
