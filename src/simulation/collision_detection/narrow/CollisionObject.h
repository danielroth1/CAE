#ifndef COLLISIONOBJECT_H
#define COLLISIONOBJECT_H

#include <data_structures/DataStructures.h>

class BoundingVolume;
class CollisionObjectVisitor;


class CollisionObject
{
public:
    enum class Type
    {
        SPHERE, TRIANGLE
    };

    CollisionObject();
    virtual ~CollisionObject();

    // Updates the world space position coordinates of the reference point,
    // e.g. applying the rotation matrix if in BODY_SPACE representation.
    // Call this method when the position changed and before calling getPosition().
    virtual void update() = 0;

    // Same as update() but with the previous position.
    virtual void updatePrevious() = 0;

    virtual void accept(CollisionObjectVisitor& visitor) = 0;

    virtual Type getType() const = 0;

    virtual Eigen::Vector getPosition() = 0;

    void setBoundingVolume(const std::shared_ptr<BoundingVolume>& boundingVolume)
    {
        mBoundingVolume = boundingVolume;
    }

    const std::shared_ptr<BoundingVolume>& getBoundingVolume() const
    {
        return mBoundingVolume;
    }

    // Returns the id of the current collision detection run. It stores the
    // id of the run in which this collision object was last seen.
    // It is used to detect if this collision object was already seen within
    // the current run. This information can be used to only update crucial
    // information of in the collision participating triangles.
    ID getRunId() const
    {
        return mRunId;
    }

    void setRunId(ID runId)
    {
        mRunId = runId;
    }

private:
    ID mRunId;
    std::shared_ptr<BoundingVolume> mBoundingVolume;
};

#endif // COLLISIONOBJECT_H
