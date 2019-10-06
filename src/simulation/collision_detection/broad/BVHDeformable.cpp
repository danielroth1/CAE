#include "BVHDeformable.h"

#include "BVSphere.h"
#include "BoundingVolumeFactory.h"

#include <simulation/collision_detection/narrow/Collider.h>
#include <simulation/collision_detection/narrow/CollisionObject.h>

#include <scene/data/geometric/Polygon.h>

#include <limits>

BVHDeformable::BVHDeformable(
        SimulationObject* so,
        Polygon* polygon,
        const std::vector<std::shared_ptr<CollisionObject>>& collisionObjects,
        BoundingVolume::Type bvType)
    : BoundingVolumeHierarchy(so, polygon, collisionObjects, bvType)
{
    initialize();
}

void BVHDeformable::initialize()
{
    initializeWithKDTree();

    // Fill mBottomToTop with nodes in the bottom to top order. This is the
    // order in which the update operations must be called in (update() must
    // have been called for all children before it can be called for the parent
    // node).
    class UpdateVisitor : public BVHNodeVisitor
    {
    public:
        UpdateVisitor(BVHDeformable& _deformable)
            : deformable(_deformable)
        {

        }

        virtual void visit(BVHChildrenNode* childrenNode)
        {
            deformable.mBottomToTop.push_back(childrenNode);

        }

        virtual void visit(BVHLeafNode* leafNode)
        {
            deformable.mBottomToTop.push_back(leafNode);
        }

    private:
        BVHDeformable& deformable;

    } visitor(*this);

    BVHTreeTraverser traverser(getRoot());
    traverser.traverse(visitor, TraverserStrategy::BOTTOM_TO_TOP);
}

void BVHDeformable::udpate()
{
    for(BVHNode* node : mBottomToTop)
    {
        if (node->isLeaf())
        {
            BVLeafData* data = static_cast<BVHLeafNode*>(node)->getData();
            data->getCollisionObject()->update();
            data->getBoundingVolume()->update(*data->getCollisionObject());
        }
        else
        {
            BVChildrenData* data = static_cast<BVHChildrenNode*>(node)->getData();
            data->getBoundingVolume()->update(data->getChild1(), data->getChild2());
        }
    }
}

void BVHDeformable::initializeWithKDTree()
{
    std::vector<std::shared_ptr<BoundingVolume>> leafSpheres;
    for (const std::shared_ptr<CollisionObject>& co : mCollisionObjects)
    {
        leafSpheres.push_back(
                    BoundingVolumeFactory::createBoundingVolume(
                        *co, *mPolygon, getBoundingVolumeType()));
    }

    class SetRootVisitor : public BVHNodeVisitor
    {
    public:
        SetRootVisitor(BVHDeformable& _deformable)
            : deformable(_deformable)
        {

        }

        virtual void visit(BVHChildrenNode* childrenNode)
        {
            deformable.setRoot(childrenNode);
        }

        virtual void visit(BVHLeafNode* leafNode)
        {
            deformable.getRoot()->addChild(leafNode);
        }

        BVHDeformable& deformable;

    } visitor(*this);

    std::shared_ptr<BoundingVolume> dummyPtr;
    BVHNode* node = initializeWithKDTreeRec(leafSpheres, mCollisionObjects, dummyPtr);
    node->accept(visitor);

//    print();

    std::cout << "Number of nodes in kd tree = " << calculateNumberOfNodes() << "\n";
}

BVHNode* BVHDeformable::initializeWithKDTreeRec(
        const std::vector<std::shared_ptr<BoundingVolume>>& boundingVolumes,
        const std::vector<std::shared_ptr<CollisionObject>>& collisionObjects,
        std::shared_ptr<BoundingVolume>& boundingVolumeRet)
{

    if (boundingVolumes.size() == 0)
        std::cout << "No spheres. This shouldn't happen!\n";
    // Recursive algorithm:
    // divide given spheres in two sets
    // if set > 1 -> ChildrenNode child = initializeWithKDTreeRec(ChildrenNode, set)
    // if set == 1 -> LeafNode, nothing to do

    // Pseudo code:
    // ChildrenNode* initializeWithKDTree(vector<BVSpheres*> spheres)
    //      ChildrenNode* parent = new ChildrenNode();
    //      left, right <- divide spheres into two sets
    //      if (left.size() == 1){
    //          LeafNode* leafNode = new LeafNode();
    //          parent->addChild(leafNode);
    //      }
    //      if (left.size() > 1){
    //          ChildrenNode* childrenNode = initializeWithKDTree(left)
    //          parent->addChild(childrenNode);
    //      }

    // Node* initializeWithKDTree(vector<BVSphere*> spheres)
    //      if (spheres.size() == 1)
    //          return new LeafNode();
    //      ChildrenNode* parent = new ChildrenNode();
    //      left, right <- divide spheres into two sets
    //      parent->addChild(initializeWithKDTree(left));
    //      parent->addChild(initializeWithKDTree(right));

    if (collisionObjects.size() == 1)
    {
        BVHLeafNode* leafNode = new BVHLeafNode("");
        BVLeafData* leafData = new BVLeafData(
                    leafNode,
                    boundingVolumes[0],
                    collisionObjects[0]);
        leafNode->setData(leafData);
        boundingVolumeRet = boundingVolumes[0];
        return leafNode;
    }

    // creates a kd-tree from the given spheres
    // returns the root sphere

    // create graph by applying space partitioning via KD-tree

    // find index of biggest axis
    Eigen::Vector min = std::numeric_limits<double>::max() * Eigen::Vector::Ones();
    Eigen::Vector max = -std::numeric_limits<double>::max() * Eigen::Vector::Ones();
    for (const std::shared_ptr<BoundingVolume>& bv : boundingVolumes)
    {
        min = min.cwiseMin(bv->getPosition());
        max = max.cwiseMax(bv->getPosition());
    }

    Eigen::Vector axisLength = max - min;

    Eigen::Index biggestAxisIndex = 0;
    for (Eigen::Index i = 1; i < 3; ++i)
    {
        if (axisLength(i) > axisLength(biggestAxisIndex))
        {
            biggestAxisIndex = i;
        }
    }

    // find center that divides spheres into two equals halfes along the longes axis
    // do this by inserting all spheres in a vector together with their position along
    // the longest axis and sort it. The first half of elements in the vector belong to
    // the left side and the second half to the right side.
    std::vector<std::tuple<
            std::shared_ptr<BoundingVolume>,
            std::shared_ptr<CollisionObject>, double>> bvPositions;

    for (size_t i = 0; i < boundingVolumes.size(); ++i)
    {
        bvPositions.push_back(std::make_tuple(
                                      boundingVolumes[i],
                                      collisionObjects[i],
                                      boundingVolumes[i]->getPosition()(biggestAxisIndex)));
    }

    std::sort(bvPositions.begin(), bvPositions.end(),
              [](const std::tuple<std::shared_ptr<BoundingVolume>, std::shared_ptr<CollisionObject>, double>& tuple1,
              const std::tuple<std::shared_ptr<BoundingVolume>, std::shared_ptr<CollisionObject>, double>& tuple2)
    {
        return std::get<2>(tuple1) < std::get<2>(tuple2);
    });

    std::vector<std::shared_ptr<BoundingVolume>> leftBv;
    std::vector<std::shared_ptr<BoundingVolume>> rightBv;
    std::vector<std::shared_ptr<CollisionObject>> leftCollisionObjects;
    std::vector<std::shared_ptr<CollisionObject>> rightCollisionObjects;

    for (size_t i = 0; i < bvPositions.size(); ++i)
    {
        if (i < bvPositions.size() / 2)
        {
            leftBv.push_back(std::get<0>(bvPositions[i]));
            leftCollisionObjects.push_back(std::get<1>(bvPositions[i]));
        }
        else
        {
            rightBv.push_back(std::get<0>(bvPositions[i]));
            rightCollisionObjects.push_back(std::get<1>(bvPositions[i]));
        }
    }

    std::shared_ptr<BoundingVolume> leftBV;
    std::shared_ptr<BoundingVolume> rightBV;

//    std::cout << "left: " << leftBv.size() << ", right: " << rightBv.size() << "\n";

    BVHNode* leftNode = initializeWithKDTreeRec(leftBv, leftCollisionObjects, leftBV);
    BVHNode* rightNode = initializeWithKDTreeRec(rightBv, rightCollisionObjects, rightBV);


    std::shared_ptr<BoundingVolume> bv =
            BoundingVolumeFactory::createBoundingVolume(
                leftBV.get(), rightBV.get(), *mPolygon, getBoundingVolumeType());
    boundingVolumeRet = bv;

    // create parent node and set data
    BVHChildrenNode* parentNode = new BVHChildrenNode("");
    parentNode->addChild(leftNode);
    parentNode->addChild(rightNode);
    BVChildrenData* data = new BVChildrenData(parentNode, bv, leftBV, rightBV);
    parentNode->setData(data);

    return parentNode;
}
