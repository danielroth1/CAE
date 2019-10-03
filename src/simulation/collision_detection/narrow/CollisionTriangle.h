#ifndef COLLISIONTRIANGLE_H
#define COLLISIONTRIANGLE_H

#include "CollisionObject.h"



class CollisionTriangle : public CollisionObject
{
public:
    CollisionTriangle();

    // CollisionObject interface
public:
    virtual Type getType() const override;
    virtual void accept(CollisionObjectVisitor& visitor) override;
    virtual Eigen::Vector getPosition() override;

};

#endif // COLLISIONTRIANGLE_H
