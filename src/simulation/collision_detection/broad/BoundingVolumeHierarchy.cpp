#include "BVSphere.h"
#include "BoundingVolume.h"
#include "BoundingVolumeHierarchy.h"

#include <memory>
#include <simulation/collision_detection/narrow/Collider.h>


BoundingVolumeHierarchy::BoundingVolumeHierarchy(SimulationObject* simulationObject,
                                                 Polygon* polygon,
                                                 const std::vector<std::shared_ptr<CollisionObject>>& collisionObjects)
    : Tree<BVChildrenData*, BVLeafData*> ("BVH")
    , mSimulationObject(simulationObject)
    , mPolygon(polygon)
    , mCollisionObjects(collisionObjects)
    , mNodeNodeDispatcher(*this)
    , mNodeLeafNodeDispatcher(*this)
{
    getRoot()->setData(nullptr);
}

void BoundingVolumeHierarchy::print()
{
    // Count numbre of total children
    class CountVisitor : public BVHNodeVisitor
    {
    public:
        CountVisitor()
        {
            nCount = 0;
        }

        virtual void visit(BVHChildrenNode* /*childrenNode*/)
        {
            nCount += 1;
        }

        virtual void visit(BVHLeafNode* /*leafNode*/)
        {
            nCount += 1;
        }

        size_t nCount;
    };

    class PrintVisitor : public BVHNodeVisitor
    {
    public:
        PrintVisitor(BoundingVolumeHierarchy& _bvh, BVHTreeTraverser& _thisTraverser)
            : bvh(_bvh)
            , thisTraverser(_thisTraverser)
        {
        }

        virtual void visit(BVHChildrenNode* childrenNode)
        {
            BVHTreeTraverser traverser(childrenNode);
            CountVisitor countVisitor;
            traverser.traverse(countVisitor);

            // print before rec
            for (size_t i = 0; i < thisTraverser.getCurrentLevel(); ++i)
            {
                std::cout << "...";
            }

            double radius = static_cast<BVSphere*>(childrenNode->getData()->getBoundingVolume())->getRadius();

            std::cout << " " << countVisitor.nCount - 1 << "(" << radius << ")" << "\n";
        }

        virtual void visit(BVHLeafNode* leafNode)
        {
            for (size_t i = 0; i < thisTraverser.getCurrentLevel(); ++i)
            {
                std::cout << "...";
            }

            double radius = static_cast<BVSphere*>(leafNode->getData()->getBoundingVolume())->getRadius();

            std::cout << " 1" << "(" << radius << ")" << "\n";
        }

        BoundingVolumeHierarchy& bvh;
        BVHTreeTraverser& thisTraverser;
    };

    BVHTreeTraverser traverser(getRoot());
    PrintVisitor pv(*this, traverser);
    traverser.traverse(pv);
}

int BoundingVolumeHierarchy::calculateNumberOfNodes()
{
    class CountVisitor : public BVHNodeVisitor
    {
    public:
        CountVisitor()
        {
            count = 0;
        }

        virtual void visit(BVHChildrenNode* /*childrenNode*/)
        {
            ++count;
        }

        virtual void visit(BVHLeafNode* /*leafNode*/)
        {
            ++count;
        }

        int count;
    } visitor;

    BVHTreeTraverser traverser(getRoot());
    traverser.traverse(visitor);

    return visitor.count;
}

bool BoundingVolumeHierarchy::collides(BoundingVolumeHierarchy* hierarchy, Collider& collider)
{
    mCollider = &collider;
    return collides(getRoot(), hierarchy->getRoot());
}

bool BoundingVolumeHierarchy::collides(BVHNode* node1, BVHNode* node2)
{
    mNodeNodeDispatcher.node1 = node1;
    node2->accept(mNodeNodeDispatcher);
    return mNodeNodeDispatcher.returnValue;
}

bool BoundingVolumeHierarchy::collides(BVHNode* node, BVHLeafNode* leafNode)
{
    mNodeLeafNodeDispatcher.leafNode = leafNode;
    node->accept(mNodeLeafNodeDispatcher);
    return mNodeLeafNodeDispatcher.returnValue;
}

bool BoundingVolumeHierarchy::collides(BVHNode* node, BVHChildrenNode* childrenNode)
{
    bool returnValue = false;
    for (size_t i = 0; i < childrenNode->getNumberOfChildren(); ++i)
    {
        if (BVHCore::getBoundingVolume(node)->intersects(childrenNode->getData()->getBoundingVolume()))
        {
            returnValue |= collides(childrenNode->getChild(static_cast<unsigned int>(i)), node);
        }
    }
    return returnValue;
}

bool BoundingVolumeHierarchy::collides(BVHChildrenNode* childrenNode, BVHLeafNode* leafNode)
{
    bool returnValue = false;
    for (size_t i = 0; i < childrenNode->getNumberOfChildren(); ++i)
    {
        if (childrenNode->getData()->getBoundingVolume()->intersects(
                leafNode->getData()->getBoundingVolume()))
        {
            returnValue |= collides(childrenNode->getChild(static_cast<unsigned int>(i)), leafNode);
        }
    }
    return returnValue;
}

bool BoundingVolumeHierarchy::collides(BVHLeafNode* leafNode1, BVHLeafNode* leafNode2)
{
    BVLeafData* data1 = leafNode1->getData();
    BVLeafData* data2 = leafNode2->getData();
    if (data1->getBoundingVolume()->intersects(data2->getBoundingVolume()))
    {
        mCollider->collides(*data1->getCollisionObject(), *data2->getCollisionObject());
        return true;
    }
    return false;
}

// NodeNodeDispatcher

BoundingVolumeHierarchy::NodeNodeDispatcher::NodeNodeDispatcher(BoundingVolumeHierarchy& _bvh)
    : bvh(_bvh)
{

}

void BoundingVolumeHierarchy::NodeNodeDispatcher::visit(BVHChildrenNode* childrenNode)
{
    returnValue = bvh.collides(node1, childrenNode);
}

void BoundingVolumeHierarchy::NodeNodeDispatcher::visit(BVHLeafNode* leafNode)
{
    returnValue = bvh.collides(node1, leafNode);
}

// NodeLeafNodeDispatcher

BoundingVolumeHierarchy::NodeLeafNodeDispatcher::NodeLeafNodeDispatcher(BoundingVolumeHierarchy& _bvh)
    : bvh(_bvh)
{

}

void BoundingVolumeHierarchy::NodeLeafNodeDispatcher::visit(BVHChildrenNode* childrenNode)
{
    returnValue = bvh.collides(childrenNode, leafNode);
}

void BoundingVolumeHierarchy::NodeLeafNodeDispatcher::visit(BVHLeafNode* leafNode1)
{
    returnValue = bvh.collides(leafNode1, leafNode);
}
