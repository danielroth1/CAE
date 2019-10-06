#ifndef COLLISIONSPHERE_H
#define COLLISIONSPHERE_H

#include "CollisionObject.h"
#include "CollisionObjectVisitor.h"

#include <simulation/references/SimulationPointRef.h>

class TopologyFeature;

class CollisionSphere : public CollisionObject
{
public:

    CollisionSphere(SimulationPointRef pointRef,
                    double radius,
                    const std::shared_ptr<TopologyFeature>& topologyFeature = nullptr);

    double getRadius() const
    {
        return mRadius;
    }

    void setRadius(double radius)
    {
        mRadius = radius;
    }

    SimulationPointRef& getPointRef()
    {
        return mPointRef;
    }

    ID getVertexIndex();

    const std::shared_ptr<TopologyFeature>& getTopologyFeature() const
    {
        return mFeature;
    }

    // CollisionObject interface
public:
    virtual void update() override;
    virtual void updatePrevious() override;

    virtual void accept(CollisionObjectVisitor& visitor) override
    {
        visitor.visit(this);
    }

    virtual Type getType() const override;

    virtual Eigen::Vector getPosition() override;

private:
    SimulationPointRef mPointRef;
    double mRadius;
    std::shared_ptr<TopologyFeature> mFeature;
};

#endif // COLLISIONSPHERE_H
