#ifndef SIMULATIONPOINTREF_H
#define SIMULATIONPOINTREF_H

#include <data_structures/DataStructures.h>
#include <data_structures/references/PointRef.h>
#include <simulation/SimulationObjectVisitor.h>

#include <memory>

class GeometricPointRef;
class Polygon;
class SimulationObject;

// Reference a point on a simulation object. Points on deformables are referenced
// by a vertex id and ponits on rigids by vectors that point from the center
// of mass (center of the polygon) to the vertex.
//
// The geometric data is used to reference the points. This means that the referenced
// point is only valid after the geometric data of the simulation object was udpated.
// This design choice was made because it allows a consistent access where the points
// are always valid and for performance reasons:
// For rigid bodies: (PositionData is in BODY_SPACE)
//  -> after a change, the world space coordinates have to be calculated only once,
//      independently of how often they are accessed.
// For deformables: (PositionData is in WORLD_SPACE)
//  -> no overhead because all points are already in world space. Deformable simulation
//      objects are operating directly on the world space geometric data, so no overhead.
class SimulationPointRef
{
public:
    // Constructor for referencign a vector w.r.t. to center of simulation object.
    // Used for deformable simulation objects.
    SimulationPointRef(SimulationObject* simObj, Polygon* polygon, Eigen::Vector r);

    // Constructor for referencing a vertex of simulation object.
    // Use for rigid bodies.
    SimulationPointRef(SimulationObject* simOb, ID index);

    // Copy constructor
    SimulationPointRef(const SimulationPointRef& ref);
//    SimulationPointRef(SimulationPointRef&& ref) = default;

    SimulationPointRef& operator=(const SimulationPointRef& ref);
//    SimulationPointRef& operator=(SimulationPointRef&& ref) = default;

    virtual ~SimulationPointRef();

    const std::shared_ptr<SimulationObject>& getSimulationObject() const;
    GeometricPointRef* getGeometricPointRef() const;

    Eigen::Vector getPoint();

    Eigen::Vector getPointPrevious();

    // Either returns the index of the referenced vertex or if no vertex is
    // referenced, returns ILLEGAL_INDEX.
    ID getIndex() const;

    // PointRef interface
public:
    virtual Eigen::Vector getGeometricPoint() const;

private:
    class GetSimulationPointDispatcher : public SimulationObjectVisitor
    {
    public:
        GetSimulationPointDispatcher(SimulationPointRef& _ref);

        virtual void visit(FEMObject& femObject)
        {
            point = ref.getPoint(femObject);
        }

        virtual void visit(SimulationPoint& sp)
        {
            point = ref.getPoint(sp);
        }

        virtual void visit(RigidBody& rigidBody)
        {
            point = ref.getPoint(rigidBody);
        }

        SimulationPointRef& ref;
        Eigen::Vector point;
    };

    class GetPrevSimulationPointDispatcher : public SimulationObjectVisitor
    {
    public:
        GetPrevSimulationPointDispatcher(SimulationPointRef& _ref);

        virtual void visit(FEMObject& femObject)
        {
            point = ref.getPreviousPoint(femObject);
        }

        virtual void visit(SimulationPoint& sp)
        {
            point = ref.getPreviousPoint(sp);
        }

        virtual void visit(RigidBody& rigidBody)
        {
            point = ref.getPreviousPoint(rigidBody);
        }

        SimulationPointRef& ref;
        Eigen::Vector point;
    };

    virtual Eigen::Vector getPoint(SimulationPoint& sp);
    virtual Eigen::Vector getPoint(RigidBody& rb);
    virtual Eigen::Vector getPoint(FEMObject& femObj);

    virtual Eigen::Vector getPreviousPoint(SimulationPoint& sp);
    virtual Eigen::Vector getPreviousPoint(RigidBody& rb);
    virtual Eigen::Vector getPreviousPoint(FEMObject& femObj);

    std::unique_ptr<GeometricPointRef> mGeometricPointRef;

    std::shared_ptr<SimulationObject> mSimulationObject;

    GetSimulationPointDispatcher mGetSimulationPointDispatcher;
    GetPrevSimulationPointDispatcher mGetPrevSimulationPointDispatcher;

};

#endif // SIMULATIONPOINTREF_H
