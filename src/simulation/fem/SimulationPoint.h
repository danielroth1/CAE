#ifndef SIMULATIONPOINT_H
#define SIMULATIONPOINT_H

#include <simulation/SimulationObject.h>

class GeometricPoint;

// A single point in a simulatoin. This simulation object is
// usually declared static. It can be used to reference declare
// single geometric points as simulatable. One application
// scenario of this is when creating a linear force that
// points to a point somewhere in the environment.
class SimulationPoint : public SimulationObject
{
public:
    SimulationPoint(Domain* domain, GeometricPoint& point);

    // SimulationObject interface
public:
    virtual SimulationObject::Type getType() const override;
    virtual void accept(SimulationObjectVisitor& visitor) override;
    virtual Eigen::Vector& getPosition(size_t id) override;
    virtual void setPosition(Eigen::Vector v, ID id) override;
    virtual void addToPosition(Eigen::Vector v, ID id) override;
    virtual void integratePositions(double stepSize) override;
    virtual void revertPositions() override;
    virtual size_t getSize() override;
    virtual GeometricData* getGeometricData() override;

private:
    GeometricPoint& mPoint;

};

#endif // SIMULATIONPOINT_H
