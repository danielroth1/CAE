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
    SimulationPoint(Domain* domain, const std::shared_ptr<GeometricPoint> point);

    // SimulationObject interface
public:
    virtual void accept(SimulationObjectVisitor& visitor) override;
    virtual void updateGeometricData(bool notifyListeners = true) override;
    virtual void applyImpulse(
            SimulationPointRef& ref, const Eigen::Vector& impulse) override;
    virtual void applyForce(
            SimulationPointRef& ref, const Eigen::Vector& force) override;
    virtual Eigen::Vector& getPosition(size_t id) override;
    virtual void setPosition(Eigen::Vector v, ID id) override;
    virtual void addToPosition(Eigen::Vector v, ID id) override;
    virtual void integratePositions(double stepSize) override;
    virtual void revertPositions() override;
    virtual void transform(const Eigen::Affine3d& transform) override;
    virtual size_t getSize() override;
    virtual double getMass() const override;
    virtual GeometricData* getGeometricData() override;

private:
    std::shared_ptr<GeometricPoint> mPoint;

};

#endif // SIMULATIONPOINT_H
