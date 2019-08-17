#ifndef FEMSIMULATION_H
#define FEMSIMULATION_H

#include "data_structures/DataStructures.h"
#include "ui/UniqueVertex.h"
#include <memory> // shared_ptr
#include <simulation/Simulation.h>

class FEMObject;
class LinearForce;
class Selection;
class SimulationPointRef;
class Truncation;

// call these methods
// initialize();
// while (...) {
//   updateForcesAndStiffnessMatrix();
//   solve();
//   update();
// }
class FEMSimulation : public Simulation
{
public:
    FEMSimulation(
            Domain* domain,
            std::shared_ptr<CollisionManager> collisionManager);

    virtual ~FEMSimulation() override;

    // Getters
    std::vector<std::shared_ptr<FEMObject>>& getFEMObjects();
    std::shared_ptr<FEMObject>& getFEMObject(int id);

    // Access methods
    void addFEMObject(std::shared_ptr<FEMObject> femObject);
    bool removeFEMObject(FEMObject* femObj);

    // Truncation methods
    // Inefficient way of adding entries to the truncation of the given
    // fem object.
    void addTruncation(FEMObject* femObject, const std::vector<ID>& vectorIDs);

    // Very inefficient O(2n^2) way of removing truncations.
    void removeTruncation(FEMObject* femObject, const std::vector<ID>& vectorIDs);
    void clearTruncation();

    void printStiffnessMatrix(FEMObject* femObject);

    // Call this before calling solve.
    void initializeStep();

    // Calculates forces and updates the velocities according to physics.
    void solve(double stepSize, bool firstStep);

    void revertSolverStep();

    // Time integrates the velocity to update the positions.
    // The previous position state can be reverted to by calling revert().
    void integratePositions(double stepSize);

    // Reverts the positions to the state before the previous integratePositions()
    // call.
    void revertPositions();

    void applyDamping();

    // Tells the renderer, that the data changed and can be visualized.
    // Call this after a position change should be visualized.
    void publish();

    // public Simulation interface
public:
    void initialize() override;

    void step(double stepSize) override;

private:

    void solveExplicitly(double stepSize);

//    std::vector<std::unique_ptr<SceneObject>> m_scene_objects;
    std::vector<std::shared_ptr<FEMObject>> mFEMObjects;

};

PROXY_CLASS_DERIVED(FEMSimulationProxy, FEMSimulation, SimulationProxy, mFEMSimulation,

            PROXY_FUNCTION(FEMSimulation, mFEMSimulation, addFEMObject,
                           PL(std::shared_ptr<FEMObject> femObject),
                           PL(femObject))

            PROXY_FUNCTION(FEMSimulation, mFEMSimulation, removeFEMObject,
                           PL(FEMObject* femObj),
                           PL(femObj))

            PROXY_FUNCTION(FEMSimulation, mFEMSimulation, addTruncation,
                           PL(FEMObject* femObject, const std::vector<ID>& vectorIDs),
                           PL(femObject, vectorIDs))

            PROXY_FUNCTION(FEMSimulation, mFEMSimulation, removeTruncation,
                           PL(FEMObject* femObject, const std::vector<ID>& vectorIDs),
                           PL(femObject, vectorIDs))

            PROXY_FUNCTION(FEMSimulation, mFEMSimulation, clearTruncation, , )

            PROXY_FUNCTION(FEMSimulation, mFEMSimulation, printStiffnessMatrix,
                           PL(FEMObject* femObject),
                           PL(femObject))
            )

#endif // FEMSIMULATION_H
