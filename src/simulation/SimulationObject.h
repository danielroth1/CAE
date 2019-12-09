#ifndef SIMULATIONOBJECT_H
#define SIMULATIONOBJECT_H

#include <data_structures/DataStructures.h>
#include <Eigen/Sparse>
#include <proxy/ProxyDefs.h>

class GeometricData;
class SimulationObjectVisitor;
class SimulationPointRef;

// A SimulationObject is a wrapper class for scene objects.
// Each Simulation type provides his own SimulationObject that must
// be used to initialize that simulation.
//
// Updating SimulationObjects:
// SimulationObjects must be informed when their data that can
// be accessed from outside changes. This approach was chosen instead
// of the observer pattern because it ovoids unnecesary loose coupling.
// The class that initalizes the Simulation already knows about the
// Simulation. Therefore, it is not necessary for it to not know about
// the SimulationObject it informs about changes.
//
// Simulations and their corresponding Simulation Objects:
//
//  Simulation           SimulationObject
//
//  FEMSimulation        FEMObject
//  <simulation_type>    <simulation_object>
class SimulationObject : public std::enable_shared_from_this<SimulationObject>
{
public:

    // This enum is used to allow to identify the class of different
    // simulation objects.
    // Use this e.g. to describe the possible SimulationObjects for
    // a provided geometric data.
    // Types are non-abstract that (not necessarily directly) derive
    // from SimulationObject.
    //
    // Note:
    // It is NOT recommended to use this instead
    // of ITTI, i.e. sth. like
    // SimulationObject* so = new FEMObject();
    // ...
    // if (so->getType() == SimulationObjectType::FEM_OBJECT)
    //      dynamic_cast<FEMObject*>(so)->...
    //
    // Instead, to avoid ITTI, use the visitor pattern, i.e.
    // class SOVisitor : public SimulationObjectVisitor
    // {
    // public:
    //      SOVIsitor()
    //      visit(FEMObject* femObj) { ... }
    //      <other_visitor_methods>
    // } visitor;
    // so->accept(visitor);
    enum Type
    {
        FEM_OBJECT, SIMULATION_POINT, RIGID_BODY
    };

    virtual Type getType() const;

    virtual void accept(SimulationObjectVisitor& visitor) = 0;

    virtual void applyImpulse(
            SimulationPointRef& ref, const Eigen::Vector& impulse) = 0;
    virtual void applyForce(
            SimulationPointRef& ref, const Eigen::Vector& force) = 0;

    // Updates the underlying geometric data according to the simulation state
    // of this object.
    // \param notifyListeners -
    virtual void updateGeometricData(bool notifyListeners = true) = 0;

    virtual Eigen::Vector& getPosition(size_t id) = 0;

    // Set the position of vector at the specified id.
    virtual void setPosition(Eigen::Vector v, ID id) = 0;
    virtual void addToPosition(Eigen::Vector v, ID id) = 0;

    virtual void integratePositions(double stepSize) = 0;
    virtual void revertPositions() = 0;

    virtual void transform(const Eigen::Affine3d& transform) = 0;

    //virtual Eigen::Vector& getExternalForce(size_t id) = 0;

    // number of positions
    virtual size_t getSize() = 0;

    virtual GeometricData* getGeometricData() = 0;

    void setFrictionDynamic(double frictionDynamic);
    double getFrictionDynamic() const;

    void setFrictionStatic(double frictionStatic);
    double getFrictionStatic() const;

    Domain* getDomain();

    virtual ~SimulationObject();

protected:
    SimulationObject(Domain* domain, Type type);

    Domain* mDomain;

    Type mType;

    double mFrictionDynamic;
    double mFrictionStatic;

};

inline SimulationObject::Type SimulationObject::getType() const
{
    return mType;
}

PROXY_CLASS(SimulationObjectProxy, SimulationObject, mS,
            PROXY_FUNCTION(SimulationObject, mS, setPosition,
                           PL(Eigen::Vector v, ID id),
                           PL(v, id))
            PROXY_FUNCTION(SimulationObject, mS, addToPosition,
                           PL(Eigen::Vector v, ID id),
                           PL(v, id))
            )

#endif // SIMULATIONOBJECT_H
