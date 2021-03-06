#ifndef BOUNDINGVOLUMEHIERARCHY_H
#define BOUNDINGVOLUMEHIERARCHY_H

#include "BoundingVolume.h"

#include "BVChildrenData.h"
#include "BVLeafData.h"
#include "BVHCore.h"

#include <boost/functional/hash.hpp>
#include <data_structures/OptimizedStack.h>
#include <data_structures/tree/Tree.h>
#include <memory>
#include <unordered_set>


class Collider;
class CollisionObject;
class MeshInterpolatorFEM;
class AbstractPolygon;
class Polygon2DAccessor;
class SimulationObject;

class BoundingVolumeHierarchy : public Tree<BVChildrenData*, BVLeafData*>
{
public:

    typedef std::tuple<SimulationObject*, ID, SimulationObject*, ID> CollisionTuple;
    typedef std::unordered_set<CollisionTuple, boost::hash<CollisionTuple>> CollisionTupleSet;

    BoundingVolumeHierarchy(SimulationObject* so,
                            AbstractPolygon* mPolygon,
                            const std::shared_ptr<MeshInterpolatorFEM>& interpolator,
                            const std::vector<std::shared_ptr<CollisionObject>>& collisionObjects,
                            BoundingVolume::Type bvType,
                            double collisionMargin);

    virtual void initialize() = 0;

    // Updates the bounding volume hierarchy.
    virtual void udpate() = 0;

    // Updates the polygon and the mesh interpolator if its there. Does not
    // notify the polygons listeners and only updates the positions and
    // face normals because those are needed for collision detection.
    void updateGeometries();

    // Prints the tree to the console.
    // For each node the number of leaf nodes are printed as well.
    // \return the number of children.
    void print();

    int calculateNumberOfNodes();

    // Cheacks if bounding volumes of this hierarcy collide with
    // bounding volumes of the given hierarchy.
    // If there are collisions, collider.collides(CollisionObject* co, CollisionObject* co)
    // is called.
    virtual bool collides(
            BoundingVolumeHierarchy* hierarchy,
            Collider& collider,
            int runId,
            const CollisionTupleSet& ignoreFeatures);

    // Iterative check for collisions.
    bool collidesIterative(
            BoundingVolumeHierarchy* hierarchy,
            int runId,
            const CollisionTupleSet& ignoreFeatures);

    // Recursive check for collisions.
    //
    // Dispatches node2, calling either
    // collides(BVHNode*, BVHChildrenNode*) or
    // collides(BVHNode*, BVHLeafNode*)
    bool collides(BVHNode* node1, BVHNode* node2)
    {
        if (node1->isLeaf() && node2->isLeaf())
        {
            return collides(static_cast<BVHLeafNode*>(node1), static_cast<BVHLeafNode*>(node2));
        }
        else if (!node1->isLeaf() && node2->isLeaf())
        {
            return collides(static_cast<BVHChildrenNode*>(node1), static_cast<BVHLeafNode*>(node2));
        }
        else if (node1->isLeaf() && !node2->isLeaf())
        {
            return collides(static_cast<BVHChildrenNode*>(node2), static_cast<BVHLeafNode*>(node1));
        }
        else
        {
            return collides(static_cast<BVHChildrenNode*>(node1), static_cast<BVHChildrenNode*>(node2));
        }
    }

    // Dispatches node, calling either
    // collides(BVHChildrenNode*, LeafNode*) or
    // collides(BVHLeafNode*, LeafNode*)
    bool collides(BVHNode* node, BVHLeafNode* leafNode);

    // Implements part B
    bool collides(BVHNode* node, BVHChildrenNode* childrenNode);

    // Implements part A
    bool collides(BVHChildrenNode* childrenNode, BVHLeafNode* leafNode);

    // Implements part C
    bool collides(BVHLeafNode* leafNode1, BVHLeafNode* leafNode2);

    BoundingVolume::Type getBoundingVolumeType() const;

    void setCollisionMargin(double collisionMargin);
    double getCollisionMargin() const;

    AbstractPolygon* getPolygon() const;
    SimulationObject* getSimulationObject() const;
    MeshInterpolatorFEM* getInterpolator() const;

protected:

    AbstractPolygon* mPolygon;
    SimulationObject* mSimulationObject;
    std::shared_ptr<MeshInterpolatorFEM> mInterpolator;

    std::vector<std::shared_ptr<CollisionObject>> mCollisionObjects;

    double mCollisionMargin;

private:

    struct StackElement
    {
        StackElement()
            : node1(nullptr)
            , node2(nullptr)
        {
        }

        StackElement(BVHNode* _node1, BVHNode* _node2)
            : node1(_node1)
            , node2(_node2)
        {
        }

        BVHNode* node1;
        BVHNode* node2;
    };

    // only valid for the duration of a call of
    // collides(BoundingVolumeHierarchy* hierarchy, Collider& collider)
    Collider* mCollider;

    BoundingVolume::Type mBvType;

    OptimizedStack<StackElement> mStack; // For iterative collision call.

    std::shared_ptr<Polygon2DAccessor> mAccessor;
};

#endif // BOUNDINGVOLUMEHIERARCHY_H
