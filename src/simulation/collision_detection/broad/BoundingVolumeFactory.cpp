#include "BVAABB.h"
#include "BVSphere.h"
#include "BoundingVolumeFactory.h"

#include <simulation/collision_detection/narrow/CollisionObjectVisitor.h>
#include <simulation/collision_detection/narrow/CollisionSphere.h>

#include <simulation/references/SimulationPointRef.h>

#include <scene/data/geometric/AbstractPolygon.h>

BoundingVolumeFactory::BoundingVolumeFactory()
{

}

std::shared_ptr<BoundingVolume> BoundingVolumeFactory::createBoundingVolume(
        CollisionObject& co,
        AbstractPolygon& polygon,
        BoundingVolume::Type bvType)
{
    switch (bvType)
    {
    case BoundingVolume::Type::AABB:
    {
        return createBVAABB(co, polygon);
    }
    case BoundingVolume::Type::SPHERE:
    {
        return createBVSphere(co, polygon);
    }
    }

    return nullptr;
}

std::shared_ptr<BoundingVolume> BoundingVolumeFactory::createBoundingVolume(
        BoundingVolume* bv1,
        BoundingVolume* bv2,
        AbstractPolygon& polygon,
        BoundingVolume::Type bvType)
{
    switch (bvType)
    {
    case BoundingVolume::Type::AABB:
    {
        return createBVAABB(static_cast<BVAABB*>(bv1),
                            static_cast<BVAABB*>(bv2),
                            polygon);
    }
    case BoundingVolume::Type::SPHERE:
    {
        return createBVSphere(static_cast<BVSphere*>(bv1),
                              static_cast<BVSphere*>(bv2),
                              polygon);
    }
    }

    return nullptr;
}

std::shared_ptr<BVSphere> BoundingVolumeFactory::createBVSphere(
        CollisionObject& co,
        AbstractPolygon& polygon)
{
    class BVSphereCreator : public CollisionObjectVisitor
    {
    public:
        BVSphereCreator(AbstractPolygon& _p)
            : p(_p)
        {

        }

        virtual void visit(CollisionSphere* collisionSphere)
        {
//            PolygonVectorRef(collisionSphere->getGeometricPointRef()->getGeometricData(),
//                             collisionSphere->getPointRef().getPoint());
            switch (p.getPositionType())
            {
            case BSWSVectors::BODY_SPACE:
                returnValue = std::make_shared<BVSphere>(collisionSphere->getPosition(),
                                           collisionSphere->getRadius());
                returnValue->setR(collisionSphere->getPosition() -
                                  p.getTransform().translation());
                break;
            case BSWSVectors::WORLD_SPACE:
                returnValue = std::make_shared<BVSphere>(collisionSphere->getPosition(),
                                           collisionSphere->getRadius());
                break;

            }
        }

        virtual void visit(CollisionTriangle* /*collisionTriangle*/)
        {
            // Nothing to do here.
        }

        std::shared_ptr<BVSphere> returnValue;
        AbstractPolygon& p;
    } creator(polygon);

    co.accept(creator);
    return creator.returnValue;
}

std::shared_ptr<BVSphere> BoundingVolumeFactory::createBVSphere(
        BVSphere* sphere1,
        BVSphere* sphere2,
        AbstractPolygon& polygon)
{
    std::shared_ptr<BVSphere> sphere = std::make_shared<BVSphere>(sphere1, sphere2);

    if (polygon.getPositionType() == BSWSVectors::BODY_SPACE)
    {
        sphere->setR(sphere->getPosition() -
                     polygon.getTransform().translation());
    }

    return sphere;
}

std::shared_ptr<BVAABB> BoundingVolumeFactory::createBVAABB(
        CollisionObject& co,
        AbstractPolygon& /*polygon*/)
{
    std::shared_ptr<BVAABB> bv = std::make_shared<BVAABB>();
    bv->update(co, 0);
    bv->updatePosition();

    return bv;
}

std::shared_ptr<BVAABB> BoundingVolumeFactory::createBVAABB(
        BVAABB* bv1,
        BVAABB* bv2,
        AbstractPolygon& /*polygon*/)
{
    std::shared_ptr<BVAABB> aabb = std::make_shared<BVAABB>();
    aabb->update(bv1, bv2);
    aabb->updatePosition();

    return aabb;
}
