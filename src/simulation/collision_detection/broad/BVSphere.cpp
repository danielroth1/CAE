#include "BVSphere.h"
#include "BoundingVolumeVisitor.h"

#include <simulation/collision_detection/narrow/CollisionSphere.h>
#include <simulation/collision_detection/narrow/CollisionTriangle.h>

#include <times/timing.h>

BVSphere::BVSphere(
        const Eigen::Vector& position,
        double radius)
    : BoundingVolume()
    , mPosition(position)
    , mRadius(radius)
    , mIntersectVisitor(*this)
{

    if (mPosition(0) != mPosition(0))
        std::cout << "illegal position\n";
}

BVSphere::BVSphere(BVSphere* sphere1, BVSphere* sphere2)
    : BoundingVolume()
    , mIntersectVisitor(*this)
{
    update(sphere1, sphere2);
}

void BVSphere::accept(BoundingVolumeVisitor& visitor)
{
    visitor.visit(this);
}

bool BVSphere::intersects(BoundingVolume* bv)
{
    bv->accept(mIntersectVisitor);
    return mIntersectVisitor.returnValue;
}

void BVSphere::update(CollisionObject& collisionObject)
{
    mPosition = collisionObject.getPosition();

    if (mPosition(0) != mPosition(0))
        std::cout << "illegal position\n";
}

void BVSphere::update(BoundingVolume* bv1, BoundingVolume* bv2)
{
//    START_TIMING_SIMULATION("BVSphere::update()");
    BVSphere* bvSphere1 = static_cast<BVSphere*>(bv1);
    BVSphere* bvSphere2 = static_cast<BVSphere*>(bv2);

    Eigen::Vector diameter = bvSphere2->getPosition() - bvSphere1->getPosition();

    if (diameter.norm() < 1e-8)
    {
        // positions are identical
        mRadius = std::max(bvSphere1->getRadius(), bvSphere2->getRadius());
        mPosition = bvSphere1->getPosition();

        if (mPosition(0) != mPosition(0))
            std::cout << "illegal position\n";
    }
    else
    {
        diameter += (bvSphere1->getRadius() + bvSphere2->getRadius()) * diameter.normalized();

        mRadius = diameter.norm() / 2;
        mPosition = bvSphere1->getPosition() + diameter / 2 - bvSphere1->getRadius() * diameter.normalized();
        if (mRadius != mRadius)
            std::cout << "illegal radius\n";
    }
//    STOP_TIMING_SIMULATION;
}

Eigen::Vector BVSphere::getPosition() const
{
    return mPosition;
}

double BVSphere::getRadius() const
{
    return mRadius;
}

void BVSphere::setRadius(double radius)
{
    mRadius = radius;
}

void BVSphere::setR(const Vector& r)
{
    mR = r;
}

void BVSphere::setPosition(const Vector& position)
{
    mPosition = position;

    if (mPosition(0) != mPosition(0))
        std::cout << "illegal position\n";
}

void BVSphere::BVIntersectsVisitor::visit(BVSphere* sphere)
{
//    START_TIMING_SIMULATION("BVSphere::BVIntersectsVisitor::visit()");
    // distance of center points is smaller than sum of radia
    returnValue = (bvSphere.getPosition() - sphere->getPosition()).norm() <
            (bvSphere.mRadius + sphere->getRadius());
//    STOP_TIMING_SIMULATION;

//    if (returnValue)
//        std::cout << "pos1 = " << bvSphere.getPosition().transpose() <<
//                     ", pos2 = " << sphere->getPosition().transpose() <<
//                     ", radius1 = " << bvSphere.getRadius() <<
//                     ", radius2 = " << bvSphere.getRadius() << "\n";
}
