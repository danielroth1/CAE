#include "FEMSimulation.h"
#include "FEMObject.h"
#include "ui/UniqueVertex.h"
#include "simulation/forces/LinearForce.h"
#include <scene/data/simulation/FEMData.h>
#include <simulation/SimulationObjectVisitor.h>
#include <simulation/constraints/Truncation.h>
#include <simulation/fem/FEMObject.h>

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
    for (ID vId : vectorIDs)
    {
        Truncation* t = femObject->getTruncation();
        if (std::find(t->getTruncatedVectorIds().begin(),
                      t->getTruncatedVectorIds().end(),
                      vId) == t->getTruncatedVectorIds().end())
        {
            femObject->getVelocities()[vId] = Vector::Zero();
            t->getTruncatedVectorIds().push_back(vId);
        }
    }
}

void FEMSimulation::removeTruncation(
        FEMObject* femObject,
        const std::vector<ID>& vectorIDs)
{
    for (ID vId : vectorIDs)
    {
        Truncation* t = femObject->getTruncation();
        auto it = std::find(t->getTruncatedVectorIds().begin(),
                            t->getTruncatedVectorIds().end(),
                            vId);
        if (it != t->getTruncatedVectorIds().end())
        {
            t->getTruncatedVectorIds().erase(it);
        }
    }
}

void FEMSimulation::clearTruncation()
{
    for (std::shared_ptr<FEMObject>& fo : mFEMObjects)
        fo->getTruncation()->getTruncatedVectorIds().clear();
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

        fo->updateFEM(true);
    }
}

void FEMSimulation::solve(double stepSize, bool firstStep)
{
    for (const std::shared_ptr<FEMObject>& fo : mFEMObjects)
    {
        fo->solveFEM(stepSize, true, firstStep);
    }
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

void FEMSimulation::publish()
{
    for (const std::shared_ptr<FEMObject>& fo : mFEMObjects)
    {
        fo->updateGeometricData();
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
    for (const std::shared_ptr<FEMObject>& fo : mFEMObjects)
    {
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
