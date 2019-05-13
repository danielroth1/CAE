#include "CollisionManager.h"
#include "CollisionManagerListener.h"

#include <simulation/collision_detection/broad/BVHDeformable.h>
#include <simulation/collision_detection/broad/BoundingVolumeHierarchy.h>
#include <scene/data/geometric/Polygon.h>
#include <scene/data/geometric/Polygon2D.h>
#include <scene/data/geometric/Polygon2DTopology.h>
#include <scene/data/geometric/Polygon3D.h>
#include <scene/data/geometric/Polygon3DTopology.h>
#include <simulation/SimulationObject.h>
#include <simulation/collision_detection/narrow/CollisionSphere.h>
#include <simulation/rigid/RigidBody.h>

#include <map>

CollisionManager::CollisionManager(Domain* domain)
    : mDomain(domain)
{
    mCollider = std::make_unique<Collider>();
}

Domain* CollisionManager::getDomain()
{
    return mDomain;
}

void CollisionManager::addSimulationObject(
        std::shared_ptr<SimulationObject> so,
        std::shared_ptr<Polygon> polygon,
        double sphereDiameter)
{
    // check if SimulationObject was already added, if so do nothing
    auto it = std::find_if(mCollisionData.begin(), mCollisionData.end(),
                        [so](const CollisionData& cd)
    {
        return cd.mSo == so;
    });

    if (it != mCollisionData.end())
    {
        std::cout << "SimulationObject is already collidable.\n";
        return;
    }

    // Create CollisionSpheres on vertices of Polygon2D or outer vertices of Polygon3D
    std::vector<std::shared_ptr<CollisionObject>> collisionObjects;
    switch (polygon->getDimensionType())
    {
    case Polygon::DimensionType::TWO_D:
    {
        std::shared_ptr<Polygon2D> p2 = std::static_pointer_cast<Polygon2D>(polygon);

        // Calculate minimum distance of each vertex
        std::map<unsigned int, double> minimumDistances = calculateMinimumDistances(
                    p2->getTopology().getFaces(), p2->getPositions());

//        double diameter = std::numeric_limits<double>::max();
//        for (auto it : minimumDistances)
//        {
//            diameter = std::min(diameter, it.second);
//        }
//        diameter *= radiusFactor;

        double diameter = sphereDiameter;

        ///////////////////////////////////////////
        // One sphere per vertex
        ///////////////////////////////////////////
        for (const TopologyVertex& v : p2->getTopology().getVertices())
        {
            collisionObjects.push_back(
                        std::make_shared<CollisionSphere>(
                            SimulationPointRef(so.get(), v.getID()),
                            diameter / 2,
                            std::make_shared<TopologyVertex>(v)));
        }


        if (so->getType() == SimulationObject::Type::RIGID_BODY)
        {
            std::shared_ptr<RigidBody> rigid = std::static_pointer_cast<RigidBody>(so);

            ///////////////////////////////////////////
            // Create Spheres on edges
            ///////////////////////////////////////////
            for (const TopologyEdge& e : p2->getTopology().getEdges())
            {
                Vector point1 = p2->getPositions()[e.getVertexIds()[0]];
                Vector point2 = p2->getPositions()[e.getVertexIds()[1]];

                double distance = (point2 - point1).norm();
                // at least one sphere per edge
                int nSpheres = std::max(2, static_cast<int>(floor(distance / diameter)));

                for (int i = 1; i < nSpheres; ++i)
                {
                    Vector spherePos = point1 + static_cast<double>(i) / nSpheres * (point2 - point1);

                    spherePos = spherePos - rigid->getPosition();

                    collisionObjects.push_back(
                                std::make_shared<CollisionSphere>(
                                    SimulationPointRef(so.get(), polygon.get(), spherePos),
                                    diameter / 2,
                                    std::make_shared<TopologyEdge>(e)));
                }
            }

            ///////////////////////////////////////////
            // Create Spheres on Triangles
            ///////////////////////////////////////////

            // CollisionSpheres should go over a whole triangle
            Vector pFirst;
            Vector pSecond;

            // go line by line along the longest side
            const Vectors& p = p2->getPositions();
            for (const TopologyFace& f : p2->getTopology().getFaces())
            {
                Vector vA = p[f.getVertexIds()[0]];
                Vector vB = p[f.getVertexIds()[1]];
                Vector vC = p[f.getVertexIds()[2]];

                Vector e1 = vB - vA;
                Vector e2 = vC - vB;
                Vector e3 = vA - vC;

                Vector eLongest;
                Vector eFirst;
                Vector eSecond;

                if (e1.norm() > e2.norm() &&
                    e1.norm() > e3.norm())
                {
                    eLongest = e1;
                    eFirst = vC - vA;
                    pFirst = vA;

                    eSecond = vC - vB;
                    pSecond = vB;
                }
                else if (e2.norm() > e1.norm() &&
                         e2.norm() > e3.norm())
                {
                    eLongest = e2;
                    eFirst = vA - vB;
                    pFirst = vB;

                    eSecond = vA - vC;
                    pSecond = vC;
                }
                else
                {
                    eLongest = e3;
                    eFirst = vB - vC;
                    pFirst = vC;

                    eSecond = vB - vA;
                    pSecond = vA;
                }

                // sphereRadius should be sphereDiameter

                // Here, spheres are not created along the edges.
                // Only on the inside of the triangle.

//                Vector normal = eLongest.cross(eLongest.cross(eFirst)).normalized();
                Vector normal = (eLongest.cross(eFirst)).cross(eLongest).normalized();
                double height = eFirst.dot(normal);
                int nLines = static_cast<int>(std::floor(height / diameter));

    //            double lineHeight = nLines / height;

                for (int line = 1; line < nLines; ++line)
                {
                    if ((eFirst - eSecond).norm() < 1e-10)
                        std::cout << "error: edges are identical\n";

                    Vector p1 = pFirst + line / static_cast<double>(nLines) * eFirst;
                    Vector p2 = pSecond + line / static_cast<double>(nLines) * eSecond;

                    int nSpheresPerLine =
                            static_cast<int>(std::floor((p2-p1).norm() / diameter));

                    if ((p2 - p1).norm() < 1e-10)
                        std::cout << "error: triangle is plane";
                    for (int i = 1; i < nSpheresPerLine; ++i)
                    {
                        Vector spherePos = p1 + i / static_cast<double>(nSpheresPerLine) * (p2 - p1);

                        spherePos = spherePos - rigid->getPosition();
                        collisionObjects.push_back(
                                    std::make_shared<CollisionSphere>(
                                        SimulationPointRef(so.get(), polygon.get(), spherePos),
                                        diameter / 2,
                                        std::make_shared<TopologyFace>(f)));
                    }
                }

            }
        }

        break;
    }
    case Polygon::DimensionType::THREE_D:
    {
        std::shared_ptr<Polygon3D> p3 = std::static_pointer_cast<Polygon3D>(polygon);

        // Calculate minimum distance of each OUTER vertex
        std::map<unsigned int, double> minimumDistances = calculateMinimumDistances(
                    p3->getTopology3D().getOuterFaces(), p3->getPositions());

        double maximumDistance = -1;
        for (size_t i = 0; i < p3->getOuterPositionIds().size(); ++i)
        {
            maximumDistance = std::max(maximumDistance, 2 * minimumDistances[static_cast<unsigned int>(i)]);
        }

        for (size_t i = 0; i < p3->getOuterPositionIds().size(); ++i)
        {
            ID index = static_cast<ID>(p3->getOuterPositionIds()[i]);
//            double radius = minimumDistances[static_cast<unsigned int>(index)] * radiusFactor;
            double radius = maximumDistance * sphereDiameter;
            collisionObjects.push_back(std::make_shared<CollisionSphere>(
                                           SimulationPointRef(so.get(), index),
                                           radius,
                                           std::make_shared<TopologyVertex>(
                                               p3->getTopology3D().getOuterTopology().getVertex(
                                               p3->getOuterPositionIds()[i]))));
        }
        break;
    }
    }

    // Create CollisionData
    CollisionData data;

    data.mSo = so;
    data.mPolygon = polygon;

    data.mBvh = std::make_shared<BVHDeformable>(so.get(), polygon.get(), collisionObjects);

    mCollisionData.push_back(data);

    std::cout << "added " << collisionObjects.size() << " CollisionObjects.\n";

    for (CollisionManagerListener* listener : mListeners)
        listener->notifySimulationObjectAdded(so);
}

bool CollisionManager::removeSimulationObject(const std::shared_ptr<SimulationObject>& so)
{
    auto it = std::find_if(mCollisionData.begin(), mCollisionData.end(),
                        [so](const CollisionData& cd)
    {
        return cd.mSo == so;
    });

    bool returnValue;
    if (it != mCollisionData.end())
    {
        mCollisionData.erase(it);
        returnValue = true;

        for (CollisionManagerListener* listener : mListeners)
            listener->notifySimulationObjectRemoved(so);
    }
    else
    {
        returnValue = false;
    }

    return returnValue;
}

bool CollisionManager::collideAll()
{
    mCollider->clear();

    bool collisionOccured = false;
    for (size_t i = 0; i < mCollisionData.size(); ++i)
    {
        for (size_t j = i+1; j < mCollisionData.size(); ++j)
        {
            collisionOccured |=
                    mCollisionData[i].mBvh->collides(
                        mCollisionData[j].mBvh.get(), *mCollider.get());
        }
    }

    for (CollisionManagerListener* listener : mListeners)
        listener->notifyCollideAllCalled();

    return collisionOccured;

//    if (mCollider->getCollisions().size() > 0)
//        std::cout << "collisions detected = " << mCollider->getCollisions().size() << "\n";
}

void CollisionManager::updateAll()
{
    for (const CollisionData& cd : mCollisionData)
    {
        cd.mBvh->udpate();
    }

    for (CollisionManagerListener* listener : mListeners)
        listener->notifyUpdateAllCalled();
}

std::shared_ptr<Collider> CollisionManager::getCollider()
{
    return mCollider;
}

bool CollisionManager::getInvertNormalsIfNecessary() const
{
    return mCollider->getInvertNormalsIfNecessary();
}

size_t CollisionManager::getNumberOfBvhs() const
{
    return mCollisionData.size();
}

void CollisionManager::setInvertNormalsIfNecessary(bool invertNormalsIfNecessary)
{
    mCollider->setInvertNormalsIfNecessary(invertNormalsIfNecessary);
}

std::shared_ptr<BoundingVolumeHierarchy> CollisionManager::getBoundingVolumeHierarchy(size_t index)
{
    return mCollisionData[index].mBvh;
}

std::shared_ptr<BoundingVolumeHierarchy> CollisionManager::getBoundingVolumeHierarchy(SimulationObject* so)
{
    auto it = std::find_if(mCollisionData.begin(), mCollisionData.end(),
                           [so](const CollisionData& cd)
    {
        return cd.mSo.get() == so;
    });

    std::shared_ptr<BoundingVolumeHierarchy> returnValue;

    if (it != mCollisionData.end())
        returnValue = it->mBvh;
    else
        returnValue = nullptr;

    return returnValue;
}

bool CollisionManager::addListener(CollisionManagerListener* listener)
{
    auto it = std::find(mListeners.begin(), mListeners.end(), listener);
    if (it == mListeners.end())
    {
        mListeners.push_back(listener);
        return true;
    }
    return false;
}

bool CollisionManager::removeListener(CollisionManagerListener* listener)
{
    auto it = std::find(mListeners.begin(), mListeners.end(), listener);
    if (it != mListeners.end())
    {
        mListeners.erase(it);
        return true;
    }
    return false;
}

std::map<unsigned int, double> CollisionManager::calculateMinimumDistances(
        const std::vector<TopologyFace>& faces,
        const Vectors& positions)
{
    std::map<unsigned int, double> distancePairs; // vertex ID, minimum distance
    for (const TopologyFace& f : faces)
    {
        // combinations of i: (0, 1), (1, 2), (2, 0)
        for (unsigned int i = 0; i < 3; ++i)
        {
            unsigned int id1 = f.getVertexIds()[i];
            unsigned int id2 = f.getVertexIds()[(i+1)%3];

            double distance = (positions[id1] - positions[id2]).norm();

            auto it = distancePairs.find(id1);
            if (it != distancePairs.end())
                it->second = std::min(it->second, distance);
            else
                distancePairs[id1] = distance;

            it = distancePairs.find(id2);
            if (it != distancePairs.end())
                it->second = std::min(it->second, distance);
            else
                distancePairs[id2] = distance;
        }
    }
    return distancePairs;
}
