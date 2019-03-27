#include "BVHDeformable.h"

#include "BVSphere.h"
#include "BoundingVolumeFactory.h"

#include <simulation/collision_detection/narrow/Collider.h>

#include <scene/data/geometric/Polygon.h>

#include <limits>

BVHDeformable::BVHDeformable(
        SimulationObject* so,
        Polygon* polygon,
        const std::vector<CollisionObject*>& collisionObjects)
    : BoundingVolumeHierarchy(so, polygon, collisionObjects)
{
    initialize();
}

void BVHDeformable::initialize()
{
    initializeWithKDTree();
}

void BVHDeformable::udpate()
{
    BVHTreeTraverser traverser(getRoot());
    class UpdateVisitor : public BVHNodeVisitor
    {
    public:
        UpdateVisitor()
        {

        }

        virtual void visit(BVHChildrenNode* childrenNode)
        {
            BVChildrenData* data = childrenNode->getData();
            data->getBoundingVolume()->update(data->getChild1(), data->getChild2());
        }

        virtual void visit(BVHLeafNode* leafNode)
        {
            BVLeafData* data = leafNode->getData();
            data->getBoundingVolume()->update(*data->getCollisionObject());
        }

    } visitor;

    traverser.traverse(visitor, TraverserStrategy::BOTTOM_TO_TOP);
}

void BVHDeformable::initializeWithKDTree()
{
    std::vector<BVSphere*> leafSpheres;
    for (CollisionObject* co : mCollisionObjects)
    {
        leafSpheres.push_back(BoundingVolumeFactory::createBVSphere(*co, *mPolygon));
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

    BVSphere* dummyPtr;
    BVHNode* node = initializeWithKDTreeRec(leafSpheres, mCollisionObjects, dummyPtr);
    node->accept(visitor);

//    print();

    std::cout << "Number of nodes in kd tree = " << calculateNumberOfNodes() << "\n";
}

BVHNode* BVHDeformable::initializeWithKDTreeRec(
        const std::vector<BVSphere*>& spheres,
        const std::vector<CollisionObject*>& collisionObjects,
        BVSphere*& boundingVolumeRet)
{

    if (spheres.size() == 0)
        std::cout << "No spheres. This shouldn't happen!\n";
    // Recursive algorithm:
    // divide given spheres in two sets
    // if set > 1 -> ChildrenNode child = initializeWithKDTreeRec(ChildrenNode, set)
    // if set == 1 -> LeafNode, nothing to do

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
                    spheres[0],
                    collisionObjects[0]);
        leafNode->setData(leafData);
        boundingVolumeRet = spheres[0];
        return leafNode;
    }

    // creates a kd-tree from the given spheres
    // returns the root sphere

    // create graph by applying space partitioning via KD-tree

    // find index of biggest axis
    Eigen::Vector min = std::numeric_limits<double>::max() * Eigen::Vector::Ones();
    Eigen::Vector max = Eigen::Vector::Zero();
    for (BVSphere* sphere : spheres)
    {
        min = min.cwiseMin(sphere->getPosition());
        max = max.cwiseMax(sphere->getPosition());
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
    std::vector<std::tuple<BVSphere*, CollisionObject*, double>> spherePositions;

    for (size_t i = 0; i < spheres.size(); ++i)
    {
        spherePositions.push_back(std::make_tuple(
                                      spheres[i],
                                      collisionObjects[i],
                                      spheres[i]->getPosition()(biggestAxisIndex)));
    }

    std::sort(spherePositions.begin(), spherePositions.end(),
              [](const std::tuple<BVSphere*, CollisionObject*, double>& tuple1,
              const std::tuple<BVSphere*, CollisionObject*, double>& tuple2)
    {
        return std::get<2>(tuple1) < std::get<2>(tuple2);
    });

    std::vector<BVSphere*> leftSpheres;
    std::vector<BVSphere*> rightSpheres;
    std::vector<CollisionObject*> leftCollisionObjects;
    std::vector<CollisionObject*> rightCollisionObjects;

    for (size_t i = 0; i < spherePositions.size(); ++i)
    {
        if (i < spherePositions.size() / 2)
        {
            leftSpheres.push_back(std::get<0>(spherePositions[i]));
            leftCollisionObjects.push_back(std::get<1>(spherePositions[i]));
        }
        else
        {
            rightSpheres.push_back(std::get<0>(spherePositions[i]));
            rightCollisionObjects.push_back(std::get<1>(spherePositions[i]));
        }
    }

    BVSphere* leftBV;
    BVSphere* rightBV;

//    std::cout << "left: " << leftSpheres.size() << ", right: " << rightSpheres.size() << "\n";

    BVHNode* leftNode = initializeWithKDTreeRec(leftSpheres, leftCollisionObjects, leftBV);
    BVHNode* rightNode = initializeWithKDTreeRec(rightSpheres, rightCollisionObjects, rightBV);


    BVSphere* bv = BoundingVolumeFactory::createBVSphere(leftBV, rightBV, *mPolygon);
    boundingVolumeRet = bv;

    // create parent node and set data
    BVHChildrenNode* parentNode = new BVHChildrenNode("");
    parentNode->addChild(leftNode);
    parentNode->addChild(rightNode);
    BVChildrenData* data = new BVChildrenData(parentNode, bv, leftBV, rightBV);
    parentNode->setData(data);

    return parentNode;
}
