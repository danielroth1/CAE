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
               std::shared_ptr<CollisionManager> collisionManager);

    virtual ~Simulation();

    virtual void initialize() = 0;
    virtual void step(double stepSize) = 0;

    // Act an external force on a simulation point.
    // This method dispatches to the other actExternalForce() methods, depending
    // on if the force is acted w.r.t. an arbitrary vector or w.r.t. a vertex with
    // index.
    void actExternalForce(SimulationPointRef* ref, Eigen::Vector force);

    Domain* getDomain();

protected:
    Domain* mDomain;

    bool mRunning;

    std::shared_ptr<CollisionManager> mCollisionManager;
};

PROXY_CLASS(SimulationProxy, Simulation, mS,
            PROXY_FUNCTION(Simulation, mS, initialize, , )
            PROXY_FUNCTION(Simulation, mS, step,
                           PL(double stepSize),
                           PL(stepSize))
//            PROXY_FUNCTION(Simulation, mS, preSimulationStep, , )
//            PROXY_FUNCTION(Simulation, mS, solveExplicitly, , )
//            PROXY_FUNCTION(Simulation, mS, solve, , )
//            PROXY_FUNCTION(Simulation, mS, update, , )

            PROXY_FUNCTION(Simulation, mS, actExternalForce,
                           PL(SimulationPointRef* ref, Eigen::Vector force),
                           PL(ref, force))
            )

#endif // SIMULATION_H
