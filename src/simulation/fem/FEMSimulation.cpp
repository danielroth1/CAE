#include "FEMSimulation.h"
#include "FEMObject.h"
#include "ui/UniqueVertex.h"
#include "simulation/forces/LinearForce.h"
#include <scene/data/simulation/FEMData.h>
#include <simulation/SimulationObjectVisitor.h>
#include <simulation/constraints/Truncation.h>
#include <simulation/fem/FEMObject.h>
#include <times/timing.h>

using namespace Eigen;

FEMSimulation::FEMSimulation(
        Domain* domain,
        std::shared_ptr<CollisionManager> collisionManager)
    : Simulation(domain, collisionManager)
{
    // load .off file
//    FEMObject so = FEMObject();
//    so.loadOff("elephant.off");
//    getFEMObjects().push_back(so);
}

FEMSimulation::~FEMSimulation()
{
}

std::vector<std::shared_ptr<FEMObject>>& FEMSimulation::getFEMObjects()
{
    return mFEMObjects;
}

std::shared_ptr<FEMObject>& FEMSimulation::getFEMObject(int id)
{
    // TODO: this only works if there are no scene objects removed.
    return mFEMObjects[static_cast<unsigned int>(id)];
}

bool FEMSimulation::removeFEMObject(FEMObject* femObj)
{
    auto it = std::find_if(
                mFEMObjects.begin(),
                mFEMObjects.end(),
                [&](const std::shared_ptr<FEMObject>& p) {
        return p.get() == femObj;
    });
    if (it != mFEMObjects.end())
    {
        mFEMObjects.erase(it);
        return true;
    }
    return false;
}

void FEMSimulation::addTruncation(
        FEMObject* femObject,
        const std::vector<ID>& vectorIDs)
{
    femObject->addTrunctionIds(vectorIDs);
}

void FEMSimulation::removeTruncation(
        FEMObject* femObject,
        const std::vector<ID>& vectorIDs)
{
    femObject->removeTrunctionIds(vectorIDs);
}

void FEMSimulation::clearTruncation()
{
    for (std::shared_ptr<FEMObject>& fo : mFEMObjects)
        fo->clearTruncation();
}

void FEMSimulation::printStiffnessMatrix(FEMObject* femObject)
{
    std::cout << femObject->getStiffnessMatrix(true) << "\n";
}

void FEMSimulation::initializeStep()
{
    for (const std::shared_ptr<FEMObject>& fo : mFEMObjects)
    {
        for (size_t i = 0; i < fo->getSize(); ++i)
        {
            fo->getExternalForce(i) = Eigen::Vector::Zero();
        }
    }
    START_TIMING_SIMULATION("SimulationControl::updateFEM()");
#pragma omp parallel for
    for (size_t i = 0; i < mFEMObjects.size(); ++i)
    {
        const std::shared_ptr<FEMObject>& fo = mFEMObjects[i];
        fo->updateFEM(true);
    }
    STOP_TIMING_SIMULATION;
}

void FEMSimulation::solve(double stepSize, bool firstStep)
{
#pragma omp parallel for
    for (size_t i = 0; i < mFEMObjects.size(); ++i)
    {
        const std::shared_ptr<FEMObject>& fo = mFEMObjects[i];
        fo->solveFEM(stepSize, true, firstStep);
    }
}

void FEMSimulation::solveVelocity(double stepSize, bool firstStep)
{
    START_TIMING_SIMULATION("SimulationControl::solveVelocityFEM()");
#pragma omp parallel for
    for (size_t i = 0; i < mFEMObjects.size(); ++i)
    {
        const std::shared_ptr<FEMObject>& fo = mFEMObjects[i];
        fo->solveVelocityFEM(stepSize, true, firstStep);
    }
    STOP_TIMING_SIMULATION;
}

void FEMSimulation::revertSolverStep()
{
    for (const std::shared_ptr<FEMObject>& fo : mFEMObjects)
    {
        fo->revertSolverStep();
    }
}

void FEMSimulation::integratePositions(double stepSize)
{
    for (const std::shared_ptr<FEMObject>& fo : mFEMObjects)
    {
        fo->integratePositions(stepSize);
    }
}

void FEMSimulation::revertPositions()
{
    for (const std::shared_ptr<FEMObject>& fo : mFEMObjects)
    {
        fo->revertPositions();
    }
}

void FEMSimulation::applyDamping()
{

}

void FEMSimulation::publish(bool notifyListeners)
{
    // Parallelization doesn't seem to be worth it.
//#pragma omp parallel for
    for (size_t i = 0; i < mFEMObjects.size(); ++i)
    {
        const std::shared_ptr<FEMObject>& fo = mFEMObjects[i];
        fo->updateGeometricData(notifyListeners);
    }
}

void FEMSimulation::addFEMObject(std::shared_ptr<FEMObject> femObject)
{
    femObject->setId(static_cast<ID>(mFEMObjects.size()));
    femObject->initializeFEM();
    mFEMObjects.push_back(femObject);
}

void FEMSimulation::initialize()
{
    // initialize FEM
//    for (auto& so_ptr : mFEMObjects)
//    {
//        FEMObject* so = so_ptr.get();
    for (size_t i = 0; i < mFEMObjects.size(); ++i)
    {
        const std::shared_ptr<FEMObject>& fo = mFEMObjects[i];
        fo->initializeFEM();
    }
}

void FEMSimulation::step(double stepSize)
{
    initializeStep();
    solve(stepSize, true);
}

void FEMSimulation::solveExplicitly(double stepSize)
{
    for (const std::shared_ptr<FEMObject>& fo : mFEMObjects)
    {
        fo->solveFEMExplicitly(stepSize, true);
    }
}
