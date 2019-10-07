#ifndef BVSPHERE_H
#define BVSPHERE_H

#include "BoundingVolume.h"
#include "BoundingVolumeVisitor.h"
#include <data_structures/DataStructures.h>
#include <scene/data/references/PolygonVectorRef.h>
#include <simulation/references/SimulationPointRef.h>


class BVSphere : public BoundingVolume
{
public:
    BVSphere(const Eigen::Vector& position,
             double radius);

    BVSphere(BVSphere* sphere1,
             BVSphere* sphere2);

    // BoundingVolume interface
public:
    virtual void accept(BoundingVolumeVisitor& visitor) override;
    virtual bool intersects(BoundingVolume* bv) override;
    virtual void update(CollisionObject& collisionObject) override;
    virtual void update(BoundingVolume* bv1, BoundingVolume* bv2) override;
    virtual BoundingVolume::Type getType() const override;
    virtual Eigen::Vector getPosition() const override;
    virtual double getSize() const override
    {
        return mR(0);
    }

    double getRadius() const;

    void setRadius(double radius);
    void setR(const Eigen::Vector& r);
    void setPosition(const Eigen::Vector& position);

private:
    class BVIntersectsVisitor : public BoundingVolumeVisitor
    {
    public:
        BVIntersectsVisitor(BVSphere& _bvSphere)
            : bvSphere(_bvSphere)
        {

        }

        // BoundingVolumeVisitor interface
    public:
        virtual void visit(BVSphere* sphere);
        virtual void visit(BVAABB* aabb);

        BVSphere& bvSphere;

        bool returnValue;

    };

    Eigen::Vector mPosition; // TODO
    Eigen::Vector mR; // TODO
    double mRadius;

    BVIntersectsVisitor mIntersectVisitor;


};

#endif // BVSPHERE_H
