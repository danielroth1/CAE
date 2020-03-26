#ifndef SIMULATIONCOLLISION_H
#define SIMULATIONCOLLISION_H

#include <simulation/collision_detection/narrow/Collision.h>

class GeometricPointRef;

// Wrapper for collision.
// Contains all simulation related relevant info.
// REFACTOR:
// The idea is to move all simulation related stuff from Collision to
// this class so the CollisionDetection can be made independent of
// the simulation.
class SimulationCollision
{
public:
    SimulationCollision();
    SimulationCollision(Collision& collision);

    const Collision& getCollision() const
    {
        return mCollision;
    }

private:

    std::shared_ptr<GeometricPointRef> generateGeometricPointRef(
            SimulationObject* so,
            const Eigen::Vector3d& point,
            const Eigen::Vector4d& bary,
            int elementId) const;

    Collision mCollision;

    std::shared_ptr<GeometricPointRef> mRefA;
    std::shared_ptr<GeometricPointRef> mRefB;
};

#endif // SIMULATIONCOLLISION_H
