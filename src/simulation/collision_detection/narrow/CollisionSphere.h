#ifndef COLLISIONSPHERE_H
#define COLLISIONSPHERE_H

#include "CollisionObject.h"

#include <simulation/references/SimulationPointRef.h>

class TopologyFeature;

class CollisionSphere : public CollisionObject
{
public:

    CollisionSphere(SimulationPointRef pointRef,
                    double radius,
                    const std::shared_ptr<TopologyFeature>& topologyFeature = nullptr);

    double getRadius() const;

    void setRadius(double radius);

    SimulationPointRef& getPointRef();

    ID getVertexIndex();

    const std::shared_ptr<TopologyFeature>& getTopologyFeature() const;

    // CollisionObject interface
public:
    virtual void accept(CollisionObjectVisitor& visitor) override;
    virtual Eigen::Vector getPosition() override;

private:
    SimulationPointRef mPointRef;
    double mRadius;
    std::shared_ptr<TopologyFeature> mFeature;
};

#endif // COLLISIONSPHERE_H
