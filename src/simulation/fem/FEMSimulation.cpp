#include "FEMSimulation.h"
#include "FEMObject.h"
#include "ui/UniqueVertex.h"
#include "simulation/constraints/LinearForce.h"
#include <scene/data/simulation/FEMData.h>
#include <simulation/SimulationObjectVisitor.h>
#include <simulation/constraints/Truncation.h>
#include <simulation/fem/FEMObject.h>

using namespace Eigen;

FEMSimulation::FEMSimulation(
        Domain* domain,
        double timeStep,
        std::shared_ptr<CollisionManager> collisionManager)
    : Simulation(domain, timeStep, collisionManager)
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

void FEMSimulation::initializeStep()
{
    for (const std::shared_ptr<FEMObject>& fo : mFEMObjects)
    {
        for (size_t i = 0; i < fo->getSize(); ++i)
        {
            fo->getExternalForce(i) = fo->getMass(i) * Vector(0, -9.81, 0);
        }

        fo->updateFEM(true);
    }

    applyLinearToExternalForces();
}

void FEMSimulation::solve(bool firstStep)
{
    for (const std::shared_ptr<FEMObject>& fo : mFEMObjects)
    {
        fo->solveFEM(mTimeStep, true, firstStep);
    }
}

void FEMSimulation::revertSolverStep()
{
    for (const std::shared_ptr<FEMObject>& fo : mFEMObjects)
    {
        fo->revertSolverStep();
    }
}

void FEMSimulation::integratePositions()
{
    for (const std::shared_ptr<FEMObject>& fo : mFEMObjects)
    {
        fo->integratePositions(mTimeStep);
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

void FEMSimulation::step()
{
    initializeStep();
    solve(true);
}

void FEMSimulation::actExternalForce(SimulationObject* so, ID vertexIndex, Vector force)
{
    class ForceActingVisitor : public SimulationObjectVisitor
    {
    public:
        ForceActingVisitor(ID _id, Vector& _force)
            : id(_id)
            , force(_force)
        {

        }

        virtual void visit(FEMObject& femObject)
        {
            femObject.getExternalForce(id) += force;
            // TODO inform about geometric data update?
//            femObject.setRequiringUpdate(true);
        }

        virtual void visit(SimulationPoint& /*sp*/)
        {
            // SimulationPoints are ignored by this simulation
        }


        virtual void visit(RigidBody& /*femObject*/)
        {
            // TODO: act external force on rigid body
        }

        ID id;
        Vector& force;
    } v(vertexIndex, force);

    so->accept(v);
}

void FEMSimulation::actExternalForce(
        SimulationObject* /*so*/,
        Vector /*r*/,
        Vector /*force*/)
{
    // TODO: not implemented yet
}

void FEMSimulation::solveExplicitly()
{
    for (const std::shared_ptr<FEMObject>& fo : mFEMObjects)
    {
        fo->solveFEMExplicitly(mTimeStep, true);
    }
}

void FEMSimulation::applyLinearToExternalForces()
{
    for (const std::shared_ptr<LinearForce>& lf : mLinearForces)
    {
//        int so_id = lf->getSourceVertex().getSimulationObjectId();
//        int vertex_id = lf->getSourceVertexId().getVertexId();
//        FEMObject* so = static_cast<FEMObject*>(mFEMObjects[static_cast<unsigned int>(so_id)]);
//        Eigen::Vector source_vertex = so->getPositions()[static_cast<unsigned int>(vertex_id)];
        // with or without normalize decides if length plays a role or not.
        Eigen::Vector force = lf->getStrength() * 100 *
                (lf->getTargetVector().getPoint() -
                 lf->getSourceVector().getPoint()).normalized();
//        SimulationObject* sourceObject = lf->getSourceSimulationObject();
                //static_cast<FEMData*>(lf->getSourceVertex().getSceneLeafData()->getSimulationData())->getFEMObject();
        // ourceObject->getExternalForce(lf->getSourceVectorID()) += force;
        const SimulationPointRef& source = lf->getSourceVector();
        const SimulationPointRef& target = lf->getTargetVector();
        Simulation::actExternalForce(source, force);
        Simulation::actExternalForce(target, -force);


    }
}
