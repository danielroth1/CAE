#ifndef SIMULATIONPOINTREF_H
#define SIMULATIONPOINTREF_H

#include <data_structures/DataStructures.h>
#include <data_structures/references/PointRef.h>
#include <scene/data/references/GeometricPointRef.h>
#include <simulation/SimulationObjectVisitor.h>

#include <memory>

class GeometricData;
class GeometricPointRefVisitor;
class Polygon;
class Polygon3D;
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
    // The update policy controls when the target point is calculated.
    // Default: ON_DEMAND
    // ON_DEMAND - the point is always calculated in the getter, e.g. getPoint()
    //      The update() method does nothing. Use this policy if getPoint()
    //      is only called once before the rotation matrix changed or if the
    //      position representation type doesn't use rotations (e.g. WORLD_SPACE).
    // ON_UPDATE_CALL - the point is only updated in the update() call. getPoint()
    //      is then just a getter with little overhead. Use this method to
    //      avoid unnecessary rotation matrix-vector multiplications when in
    //      BODY_SPACE representation by calling update() only once when the
    //      rotation matrix or position changed.
    enum class UpdatePolicy
    {
        ON_DEMAND, ON_UPDATE_CALL
    };

    // Constructor for referencign a vector w.r.t. to center of simulation object.
    // Used for deformable simulation objects.
    SimulationPointRef(SimulationObject* simObj, Polygon* polygon, Eigen::Vector r);

    // Constructor for referencing a vertex of simulation object.
    // Use for rigid bodies.
    SimulationPointRef(SimulationObject* simObj, ID index);

    // Constructor for referencing a vertex of simulation object.
    // Use for rigid bodies.
    SimulationPointRef(SimulationObject* simObn,
                       Polygon3D* polygon,
                       const std::array<double, 4>& bary,
                       ID elementId);

    // Copy constructor
    SimulationPointRef(const SimulationPointRef& ref);
//    SimulationPointRef(SimulationPointRef&& ref) = default;

    SimulationPointRef& operator=(const SimulationPointRef& ref);
//    SimulationPointRef& operator=(SimulationPointRef&& ref) = default;

    virtual ~SimulationPointRef();

    // Applies a given impulse
    void applyImpulse(const Eigen::Vector& impulse);

    // Calculates and returns the speed.
    Eigen::Vector calculateSpeed();

    void update();
    void updatePrevious();

    void setUpdatePolicy(UpdatePolicy policy);

    const std::shared_ptr<SimulationObject>& getSimulationObject() const;
    GeometricPointRef* getGeometricPointRef() const;
    GeometricData* getGeometricData() const;

    Eigen::Vector getPoint();

    Eigen::Vector getPointPrevious();

    // GeometricPointRef delegated methods

    GeometricPointRef::Type getGeometricType() const;

    // Either returns the index of the referenced vertex or if no vertex is
    // referenced, returns ILLEGAL_INDEX.
    ID getIndex() const;

    void accept(GeometricPointRefVisitor& visitor);

    Eigen::Vector getGeometricPoint() const;

private:

    virtual Eigen::Vector getPoint(SimulationPoint& sp);
    virtual Eigen::Vector getPoint(RigidBody& rb);
    virtual Eigen::Vector getPoint(FEMObject& femObj);

    virtual Eigen::Vector getPreviousPoint(SimulationPoint& sp);
    virtual Eigen::Vector getPreviousPoint(RigidBody& rb);
    virtual Eigen::Vector getPreviousPoint(FEMObject& femObj);

    // Calculates and returns the current point
    Eigen::Vector calculatePoint();
    Eigen::Vector calculatePointPrevious();

    std::unique_ptr<GeometricPointRef> mGeometricPointRef;

    std::shared_ptr<SimulationObject> mSimulationObject;

    Eigen::Vector mPoint; // Only used if mUpdatePolicy == ON_UPDATE_CALL
    Eigen::Vector mPointPrevious; // Only used if mUpdatePolicy == ON_UPDATE_CALL
    UpdatePolicy mUpdatePolicy;

};

#endif // SIMULATIONPOINTREF_H
