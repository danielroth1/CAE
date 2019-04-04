#ifndef COLLISIONSPHERE_H
#define COLLISIONSPHERE_H

#include "CollisionObject.h"

#include <simulation/references/SimulationPointRef.h>



class CollisionSphere : public CollisionObject
{
public:
    CollisionSphere(SimulationPointRef pointRef,
                    double radius);

    double getRadius() const;

    void setRadius(double radius);

    SimulationPointRef& getPointRef();

    ID getVertexIndex();

    // CollisionObject interface
public:
    virtual void accept(CollisionObjectVisitor& visitor) override;
    virtual Eigen::Vector getPosition() override;

private:
    SimulationPointRef mPointRef;
    double mRadius;
};

#endif // COLLISIONSPHERE_H
