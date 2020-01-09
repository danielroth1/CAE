#include "BoundingVolumeHierarchy.h"
#include "BoundingVolume.h"

#include <memory>
#include <simulation/collision_detection/narrow/Collider.h>


BoundingVolumeHierarchy::BoundingVolumeHierarchy(
        SimulationObject* simulationObject,
        Polygon* polygon,
        MeshInterpolatorFEM* interpolator,
        const std::vector<std::shared_ptr<CollisionObject>>& collisionObjects,
        BoundingVolume::Type bvType,
        double collisionMargin)
    : Tree<BVChildrenData*, BVLeafData*> ("BVH")
    , mPolygon(polygon)
    , mSimulationObject(simulationObject)
    , mInterpolator(interpolator)
    , mCollisionObjects(collisionObjects)
    , mCollisionMargin(collisionMargin)
    , mBvType(bvType)
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

            double size = childrenNode->getData()->getBoundingVolumePtr()->getSize();

            std::cout << " " << countVisitor.nCount - 1 << "(" << size << ")" << "\n";
        }

        virtual void visit(BVHLeafNode* leafNode)
        {
            for (size_t i = 0; i < thisTraverser.getCurrentLevel(); ++i)
            {
                std::cout << "...";
            }

            double size = leafNode->getData()->getBoundingVolumePtr()->getSize();

            std::cout << " 1" << "(" << size << ")" << "\n";
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

bool BoundingVolumeHierarchy::collides(BoundingVolumeHierarchy* hierarchy,
                                       Collider& collider)
{
    mCollider = &collider;
    return collidesIterative(hierarchy);
}

bool BoundingVolumeHierarchy::collidesIterative(BoundingVolumeHierarchy* hierarchy)
{
    mCollider->prepare(mPolygon, hierarchy->getPolygon(),
                       mSimulationObject, hierarchy->getSimulationObject(),
                       mInterpolator, hierarchy->getInterpolator());

    bool collides = false;

    StackElement& first = mStack.push();
    first.node1 = getRoot();
    first.node2 = hierarchy->getRoot();

    while (!mStack.empty())
    {
        StackElement& el = mStack.top();
        mStack.pop();

        BoundingVolume* bv1 = BVHCore::getBoundingVolume(el.node1);
        BoundingVolume* bv2 = BVHCore::getBoundingVolume(el.node2);

        if (bv1->intersects(bv2))
        {
            if (el.node1->isLeaf() && el.node2->isLeaf())
            {
                collides |= mCollider->collides(
                            *static_cast<BVHLeafNode*>(el.node1)->getData()->getCollisionObject(),
                            *static_cast<BVHLeafNode*>(el.node2)->getData()->getCollisionObject());
            }
            else
            {
                bool ordered;
                BVHNode* searchedNode;
                BVHNode* otherNode;
                if (el.node1->isLeaf())
                {
                    ordered = false;
                    searchedNode = el.node2;
                    otherNode = el.node1;
                }
                else if (el.node2->isLeaf())
                {
                    ordered = true;
                    searchedNode = el.node1;
                    otherNode = el.node2;
                }
                else
                {
                    // take children node with bigger bvh next
                    if (bv1->getSize() > bv2->getSize())
                    {
                        ordered = true;
                        searchedNode = el.node1;
                        otherNode = el.node2;
                    }
                    else
                    {
                        ordered = false;
                        searchedNode = el.node2;
                        otherNode = el.node1;
                    }
                }

                BVHChildrenNode* childrenNode = static_cast<BVHChildrenNode*>(searchedNode);
                for (size_t i = 0; i < searchedNode->getNumberOfChildren(); ++i)
                {
                    StackElement& elTemp = mStack.push();
                    if (ordered)
                    {
                        elTemp.node1 = childrenNode->getChild(static_cast<unsigned int>(i));
                        elTemp.node2 = otherNode;
                    }
                    else
                    {
                        elTemp.node1 = otherNode;
                        elTemp.node2 = childrenNode->getChild(static_cast<unsigned int>(i));
                    }
                }
            }

        }
    }

    return mCollider->evaluate();
}

bool BoundingVolumeHierarchy::collides(BVHNode* node, BVHLeafNode* leafNode)
{
    if (node->isLeaf())
    {
        return collides(static_cast<BVHLeafNode*>(node), leafNode);
    }
    else
    {
        return collides(static_cast<BVHChildrenNode*>(node), leafNode);
    }
}

bool BoundingVolumeHierarchy::collides(BVHNode* node, BVHChildrenNode* childrenNode)
{
    bool returnValue = false;
    for (size_t i = 0; i < childrenNode->getNumberOfChildren(); ++i)
    {
        if (BVHCore::getBoundingVolume(node)->intersects(
                childrenNode->getData()->getBoundingVolumePtr()))
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
                leafNode->getData()->getBoundingVolumePtr()))
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
    if (data1->getBoundingVolume()->intersects(data2->getBoundingVolumePtr()))
    {
        mCollider->collides(*data1->getCollisionObject(), *data2->getCollisionObject());
        return true;
    }
    return false;
}

BoundingVolume::Type BoundingVolumeHierarchy::getBoundingVolumeType() const
{
    return mBvType;
}

void BoundingVolumeHierarchy::setCollisionMargin(double collisionMargin)
{
    mCollisionMargin = collisionMargin;
}

double BoundingVolumeHierarchy::getCollisionMargin() const
{
    return mCollisionMargin;
}

Polygon* BoundingVolumeHierarchy::getPolygon() const
{
    return mPolygon;
}

SimulationObject* BoundingVolumeHierarchy::getSimulationObject() const
{
    return mSimulationObject;
}

MeshInterpolatorFEM* BoundingVolumeHierarchy::getInterpolator() const
{
    return mInterpolator;
}
