#include "BVSphere.h"
#include "BoundingVolumeFactory.h"

#include <simulation/collision_detection/narrow/CollisionObjectVisitor.h>
#include <simulation/collision_detection/narrow/CollisionSphere.h>

#include <simulation/references/SimulationPointRef.h>

#include <scene/data/geometric/Polygon.h>

BoundingVolumeFactory::BoundingVolumeFactory()
{

}

BVSphere* BoundingVolumeFactory::createBVSphere(CollisionObject& co, Polygon& polygon)
{
    class BVSphereCreator : public CollisionObjectVisitor
    {
    public:
        BVSphereCreator(Polygon& _p)
            : p(_p)
        {

        }

        virtual void visit(CollisionSphere* collisionSphere)
        {
//            PolygonVectorRef(collisionSphere->getPointRef().getGeometricPointRef()->getGeometricData(),
//                             collisionSphere->getPointRef().getPoint());
            switch (p.getPositionType())
            {
            case BSWSVectors::BODY_SPACE:
                returnValue = new BVSphere(collisionSphere->getPosition(),
                                           collisionSphere->getRadius());
                returnValue->setR(collisionSphere->getPosition() -
                                  p.getTransform().translation());
                break;
            case BSWSVectors::WORLD_SPACE:
                returnValue = new BVSphere(collisionSphere->getPosition(),
                                           collisionSphere->getRadius());
                break;

            }


        }

        virtual void visit(CollisionTriangle* /*collisionTriangle*/)
        {
            // TODO: implement this
        }

        BVSphere* returnValue;
        Polygon& p;
    } creator(polygon);

    co.accept(creator);
    return creator.returnValue;
}

BVSphere* BoundingVolumeFactory::createBVSphere(BVSphere* sphere1, BVSphere* sphere2, Polygon& polygon)
{
    BVSphere* sphere = new BVSphere(sphere1, sphere2);

    if (polygon.getPositionType() == BSWSVectors::BODY_SPACE)
    {
        sphere->setR(sphere->getPosition() -
                     polygon.getTransform().translation());
    }

    return sphere;
}
