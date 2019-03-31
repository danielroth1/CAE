#ifndef BOUNDINGVOLUMEHIERARCHY_H
#define BOUNDINGVOLUMEHIERARCHY_H

#include "BVChildrenData.h"
#include "BVLeafData.h"
#include "BVHCore.h"

#include <data_structures/tree/Tree.h>
#include <memory>


class Collider;
class CollisionObject;
class Polygon;
class SimulationObject;

class BoundingVolumeHierarchy : public Tree<BVChildrenData*, BVLeafData*>
{
public:
    BoundingVolumeHierarchy(SimulationObject* so,
                            Polygon* mPolygon,
                            const std::vector<std::shared_ptr<CollisionObject>>& collisionObjects);

    virtual void initialize() = 0;

    // Updates the bounding volume hierarchy.
    virtual void udpate() = 0;

    // Prints the tree to the console.
    // For each node the number of leaf nodes are printed as well.
    // \return the number of children.
    void print();

    int calculateNumberOfNodes();

    // Cheacks if bounding volumes of this hierarcy collide with
    // bounding volumes of the given hierarchy.
    // If there are collisions, collider.collides(CollisionObject* co, CollisionObject* co)
    // is called.
    virtual bool collides(BoundingVolumeHierarchy* hierarchy, Collider& collider);

    // Dispatches node2, calling either
    // collides(BVHNode*, BVHChildrenNode*) or
    // collides(BVHNode*, BVHLeafNode*)
    bool collides(BVHNode* node1, BVHNode* node2);

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
protected:

    SimulationObject* mSimulationObject;

    Polygon* mPolygon;

    std::vector<std::shared_ptr<CollisionObject>> mCollisionObjects;

private:

    class NodeNodeDispatcher : public BVHNodeVisitor
    {
    public:
        NodeNodeDispatcher(BoundingVolumeHierarchy& _bvh);

        virtual void visit(BVHChildrenNode* childrenNode) override;

        virtual void visit(BVHLeafNode* leafNode) override;

        BoundingVolumeHierarchy& bvh;
        BVHNode* node1;
        bool returnValue;
    };

    class NodeLeafNodeDispatcher : public BVHNodeVisitor
    {
    public:
        NodeLeafNodeDispatcher(BoundingVolumeHierarchy& _bvh);

        virtual void visit(BVHChildrenNode* childrenNode) override;

        virtual void visit(BVHLeafNode* leafNode1) override;

        BoundingVolumeHierarchy& bvh;
        BVHLeafNode* leafNode;
        bool returnValue;
    };

    NodeNodeDispatcher mNodeNodeDispatcher;
    NodeLeafNodeDispatcher mNodeLeafNodeDispatcher;

    // only valid for the duration of a call of
    // collides(BoundingVolumeHierarchy* hierarchy, Collider& collider)
    Collider* mCollider;
};

#endif // BOUNDINGVOLUMEHIERARCHY_H
