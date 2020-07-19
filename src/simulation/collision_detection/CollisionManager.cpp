#include "CollisionManager.h"
#include "CollisionManagerListener.h"

#include <simulation/collision_detection/broad/BVHDeformable.h>
#include <simulation/collision_detection/broad/BoundingVolumeHierarchy.h>
#include <scene/data/geometric/MeshInterpolatorFEM.h>
#include <scene/data/geometric/Polygon.h>
#include <scene/data/geometric/Polygon2D.h>
#include <scene/data/geometric/Polygon2DTopology.h>
#include <scene/data/geometric/Polygon3D.h>
#include <scene/data/geometric/Polygon3DTopology.h>
#include <simulation/SimulationObject.h>
#include <simulation/collision_detection/narrow/CollisionSphere.h>
#include <simulation/collision_detection/narrow/CollisionTriangle.h>
#include <simulation/rigid/RigidBody.h>
#include <times/timing.h>

#include <map>

CollisionManager::CollisionManager(Domain* domain)
    : mDomain(domain)
{
    mCollider = std::make_unique<Collider>(4e-2);
    mForceUpdate = false;
    mRunId = 0;
}

Domain* CollisionManager::getDomain()
{
    return mDomain;
}

bool CollisionManager::addSimulationObjectTriangles(
        const std::shared_ptr<SimulationObject>& so,
        const std::shared_ptr<Polygon>& polygon,
        const std::shared_ptr<MeshInterpolatorFEM>& interpolator)
{
    if (isCollidable(so))
        return false;

    std::shared_ptr<Polygon2DAccessor> accessor = polygon->createAccessor();
    const Polygon2DTopology& topology = accessor->getTopology2D();
    std::vector<std::shared_ptr<CollisionObject>> collisionObjects;

    for (size_t i = 0; i <  topology.getFacesIndices().size(); ++i)
    {
        const Face& f = topology.getFacesIndices()[i];
        collisionObjects.push_back(
                    std::make_shared<CollisionTriangle>(accessor, f, i));
    }

    return addSimulationObject(so, polygon, collisionObjects, interpolator);
}

bool CollisionManager::addSimulationObjectTriangles(
        const std::shared_ptr<SimulationObject>& so,
        const std::shared_ptr<MeshInterpolatorFEM>& interpolator)
{
    return addSimulationObjectTriangles(so, interpolator->getTarget(), interpolator);
}

bool CollisionManager::addSimulationObject(
        std::shared_ptr<SimulationObject> so,
        std::shared_ptr<Polygon> polygon,
        double sphereDiameter)
{
    // check if SimulationObject was already added, if so do nothing
    if (isCollidable(so))
        return false;

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

                    spherePos = p2->getTransform().linear().inverse() * spherePos;

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

                    Vector v1 = pFirst + line / static_cast<double>(nLines) * eFirst;
                    Vector v2 = pSecond + line / static_cast<double>(nLines) * eSecond;

                    int nSpheresPerLine =
                            static_cast<int>(std::floor((v2-v1).norm() / diameter));

                    if ((v2 - v1).norm() < 1e-10)
                        std::cout << "error: triangle is plane";
                    for (int i = 1; i < nSpheresPerLine; ++i)
                    {
                        Vector spherePos = v1 + i / static_cast<double>(nSpheresPerLine) * (v2 - v1);

                        spherePos = spherePos - rigid->getPosition();

                        spherePos = p2->getTransform().linear().inverse() * spherePos;

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
                    p3->getTopology3D().getOuterFaces(), p3->calcualtePositions2DFrom3D());

        double maximumDistance = -1;
        for (size_t i = 0; i < p3->getOuterPositionIds().size(); ++i)
        {
            maximumDistance = std::max(maximumDistance,
                                       2 * minimumDistances[
                                       static_cast<unsigned int>(i)]);
        }

        for (size_t i = 0; i < p3->getOuterPositionIds().size(); ++i)
        {
            ID index = static_cast<ID>(p3->getOuterPositionIds()[i]);
//            double radius = minimumDistances[static_cast<unsigned int>(index)] * radiusFactor;
            double radius = maximumDistance * 0.2;//sphereDiameter;
            collisionObjects.push_back(
                        std::make_shared<CollisionSphere>(
                            SimulationPointRef(so.get(), index),
                            radius,
                            std::make_shared<TopologyVertex>(
                                p3->getTopology3D().getOuterTopology().getVertex(i))
//                               std::make_shared<TopologyVertex>(
//                                   p3->getTopology3D().getOuterTopology().getVertex(
//                                   p3->getOuterPositionIds()[i]))
                                           ));
        }
        break;
    }
    }

    return addSimulationObject(so, polygon, collisionObjects, nullptr);
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

    // all kept collisions constraints that reference the removed simulation
    // object are removed.
    std::vector<SimulationCollision> validCollisions;
    for (const SimulationCollision& sc : mSimulationCollisions)
    {
        if (sc.getCollision().getSimulationObjectA() != so.get() &&
                sc.getCollision().getSimulationObjectB() != so.get())
        {
            validCollisions.push_back(sc);
        }
    }
    mSimulationCollisions = validCollisions;

    return returnValue;
}

void CollisionManager::addCollisionGroupId(
        const std::shared_ptr<SimulationObject>& so, int groupId)
{
    for (CollisionData& cd : mCollisionData)
    {
        if (cd.mSo == so)
        {
            cd.mCollisionGroups.push_back(groupId);
            std::sort(cd.mCollisionGroups.begin(), cd.mCollisionGroups.end());
            break;
        }
    }
}

void CollisionManager::setCollisionGroupIds(
        const std::shared_ptr<SimulationObject>& so,
        const std::vector<int>& groupIds)
{
    for (CollisionData& cd : mCollisionData)
    {
        if (cd.mSo == so)
        {
            cd.mCollisionGroups = groupIds;
            std::sort(cd.mCollisionGroups.begin(), cd.mCollisionGroups.end());
            break;
        }
    }
}

bool CollisionManager::collideAll()
{
    // Give this run a unique id. No other triangle will have this id.
    // It is used to distinguish already visited from non-visited triangles.
    ++mRunId;

    mCollider->clear();

    bool collisionOccured = false;

    for (size_t i = 0; i < mCollisionData.size(); ++i)
    {
        for (size_t j = i+1; j < mCollisionData.size(); ++j)
        {
            if (!isSharingCollisionGroups(
                        mCollisionData[i].mCollisionGroups,
                        mCollisionData[j].mCollisionGroups))
            {
                collisionOccured |=
                        mCollisionData[i].mBvh->collides(
                            mCollisionData[j].mBvh.get(), *mCollider.get(), mRunId,
                            mAlreadySeenCollisions);
            }
        }
    }

    size_t numContactsBefore = mSimulationCollisions.size();
    mSimulationCollisions.resize(numContactsBefore + mCollider->getCollisions().size());
    for (size_t i = numContactsBefore; i < numContactsBefore + mCollider->getCollisions().size(); ++i)
    {
        Collision& c = mCollider->getCollisions()[i - numContactsBefore];
        new (&mSimulationCollisions[i]) SimulationCollision(c);
    }
//    filterCollisions(numContactsBefore, 1);

    mNumContacts = mSimulationCollisions.size();

    for (CollisionManagerListener* listener : mListeners)
        listener->notifyCollideAllCalled();

    return collisionOccured;
}

void CollisionManager::updateAll()
{
    int updateCounter = 0;

#pragma omp parallel for if (mCollisionData.size() > 8)
    for (size_t i = 0; i < mCollisionData.size(); ++i)
    {
        CollisionData& cd = mCollisionData[i];
        bool dirty = true;

        // Only update a collision hierarchy if position or orientation of
        // the rigid changed. This is especially important to prevent
        // unnecessary updates of static rigids.
        if (!mForceUpdate && cd.mSo->getType() == SimulationObject::Type::RIGID_BODY)
        {
            RigidBody* rb = static_cast<RigidBody*>(cd.mSo.get());
            if (cd.mQ.isApprox(rb->getOrientation()) &&
                    cd.mX.isApprox(rb->getPosition()))
            {
                dirty = false;
            }
            else
            {
                cd.mQ = rb->getOrientation();
                cd.mX = rb->getPosition();
            }
        }
        if (dirty)
        {
            cd.mBvh->udpate();
            ++updateCounter;
        }
    }

    mForceUpdate = false;

    for (CollisionManagerListener* listener : mListeners)
        listener->notifyUpdateAllCalled();
}

void CollisionManager::updateGeometries()
{
    for (const CollisionData& cd : mCollisionData)
    {
        cd.mBvh->updateGeometries();
    }
}

void CollisionManager::revalidateCollisions()
{
    std::vector<SimulationCollision> validCollisions;
    SimulationCollision newSimulationCollision;
    for (SimulationCollision& c : mSimulationCollisions)
    {
        const Collision& col = c.getCollision();
        bool collides = false;

        Polygon* polyA = col.getInterpolatorA() ?
                    col.getInterpolatorA()->getTarget().get() :
                    static_cast<Polygon*>(col.getSimulationObjectA()->getGeometricData());
        Polygon* polyB = col.getInterpolatorB() ?
                    col.getInterpolatorB()->getTarget().get() :
                    static_cast<Polygon*>(col.getSimulationObjectB()->getGeometricData());

        if (col.getTopologyFeatureA()->getType() == TopologyFeature::Type::VERTEX &&
                col.getTopologyFeatureB()->getType() == TopologyFeature::Type::FACE)
        {
            // vertex-face
            collides = mCollider->collides(
                        *static_cast<TopologyVertex*>(col.getTopologyFeatureA()),
                        *static_cast<TopologyFace*>(col.getTopologyFeatureB()),
                        col.getSimulationObjectA(),
                        col.getSimulationObjectB(),
                        col.getInterpolatorA(),
                        col.getInterpolatorB(),
                        polyA,
                        polyB,
                        false,
                        newSimulationCollision.getCollision());
        }
        else if (col.getTopologyFeatureA()->getType() == TopologyFeature::Type::FACE &&
                 col.getTopologyFeatureB()->getType() == TopologyFeature::Type::VERTEX)
        {
            // face-vertex
            collides = mCollider->collides(
                        *static_cast<TopologyVertex*>(col.getTopologyFeatureB()),
                        *static_cast<TopologyFace*>(col.getTopologyFeatureA()),
                        col.getSimulationObjectB(),
                        col.getSimulationObjectA(),
                        col.getInterpolatorB(),
                        col.getInterpolatorA(),
                        polyB,
                        polyA,
                        true,
                        newSimulationCollision.getCollision());
        }
        else
        {
            // edge-edge
            collides = mCollider->collides(
                        *static_cast<TopologyEdge*>(col.getTopologyFeatureA()),
                        *static_cast<TopologyEdge*>(col.getTopologyFeatureB()),
                        col.getSimulationObjectA(),
                        col.getSimulationObjectB(),
                        col.getInterpolatorA(),
                        col.getInterpolatorB(),
                        polyA,
                        polyB,
                        newSimulationCollision.getCollision());
        }
        if (collides)
        {
            validCollisions.push_back(newSimulationCollision);
        }
    }
    mSimulationCollisions = validCollisions;
//    filterCollisions(0, 1);

    // fill already seen collisions so they are filtered out in the upcoming collision
    // detections.
    mAlreadySeenCollisions.clear();
    for (const SimulationCollision& simCol : mSimulationCollisions)
    {
        const Collision& c = simCol.getCollision();
        mAlreadySeenCollisions.insert(std::make_tuple(
                                          c.getSimulationObjectA(), c.getTopologyFeatureA()->getGeometryID(),
                                          c.getSimulationObjectB(), c.getTopologyFeatureB()->getGeometryID()));

    }
}

std::shared_ptr<Collider> CollisionManager::getCollider()
{
    return mCollider;
}

const std::vector<SimulationCollision>& CollisionManager::getCollisions() const
{
    return mSimulationCollisions;
}

bool CollisionManager::isCollidable(const std::shared_ptr<SimulationObject>& so)
{
    auto it = std::find_if(mCollisionData.begin(), mCollisionData.end(),
                        [so](const CollisionData& cd)
    {
        return cd.mSo == so;
    });

    return it != mCollisionData.end();
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

void CollisionManager::setCollisionMargin(double collisionMargin)
{
    // Update bounding volume
    for (CollisionData& data : mCollisionData)
    {
        data.mBvh->setCollisionMargin(collisionMargin);
    }
    mForceUpdate = true;

    // Update Collider
    mCollider->setCollisionMargin(collisionMargin);
}

double CollisionManager::getCollisionMargin() const
{
    return mCollider->getCollisionMargin();
}

size_t CollisionManager::getNumContacts() const
{
    return mNumContacts;
}

bool CollisionManager::addSimulationObject(
        const std::shared_ptr<SimulationObject>& so,
        const std::shared_ptr<Polygon>& polygon,
        const std::vector<std::shared_ptr<CollisionObject>>& collisionObjects,
        const std::shared_ptr<MeshInterpolatorFEM>& interpolator)
{
    if (isCollidable(so))
        return false;

    // Create CollisionData
    CollisionData data;

    data.mSo = so;
    data.mPolygon = polygon;

    data.mBvh = std::make_shared<BVHDeformable>(
                so.get(),
                polygon.get(),
                interpolator,
                collisionObjects,
                BoundingVolume::Type::AABB,
                getCollisionMargin());

    mCollisionData.push_back(data);

    std::cout << "Added " << collisionObjects.size() << " CollisionObjects.\n";

    for (CollisionManagerListener* listener : mListeners)
        listener->notifySimulationObjectAdded(so);

    return true;
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

void CollisionManager::filterCollisions(size_t offset, size_t numAllowed)
{
    size_t sizeBefore = mSimulationCollisions.size();
    size_t numTarget = mSimulationCollisions.size() - offset;

    std::vector<bool> filteredOut;
    filteredOut.resize(numTarget, false);

    for (size_t i = 0; i < numTarget; ++i)
    {
        SimulationCollision& sc = mSimulationCollisions[offset + i];

        if (sc.getCollision().isInside())
        {
            filteredOut[i] = true;
            sc.mTempValue = i;
        }
        else
        {
            filteredOut[i] = false;
        }
    }


    // fill in buckets
    typedef std::pair<SimulationObject*, SimulationObject*> SOPair;
    std::map<SOPair, std::vector<SimulationCollision*>> soMap;
    for (size_t i = 0; i < numTarget; ++i)
    {
        SimulationCollision& sc = mSimulationCollisions[offset + i];
        Collision& c = sc.getCollision();
        if (c.isInside())
        {
            SimulationObject* so1 = c.getSimulationObjectA();
            SimulationObject* so2 = c.getSimulationObjectB();

            SOPair pair = so1 < so2 ? SOPair(so1, so2) : SOPair(so2, so1);
            auto it = soMap.find(pair);
            if (it == soMap.end())
            {
                soMap[pair] = std::vector<SimulationCollision*>();

            }
            soMap[pair].push_back(&sc);
        }
    }

    for (auto& it : soMap)
    {
        std::sort(it.second.begin(), it.second.end(),
                  [](const SimulationCollision* sc1,
                  const SimulationCollision* sc2)
        {
            return sc1->getCollision().getDepth() < sc2->getCollision().getDepth();
        });
        for (size_t i = 0; i < numAllowed && i < it.second.size(); ++i)
        {
            filteredOut[(it.second[i])->mTempValue] = false;
        }
    }

    // assemble ignoring all the filtered out collisions
    size_t added = 0;
    for (size_t i = 0; i < numTarget; ++i)
    {
        if (!filteredOut[i])
        {
            mSimulationCollisions[offset + added] = mSimulationCollisions[offset + i];
            ++added;
        }
    }

    // Reduces the number of collisions that are ignores (offset) + the ones that
    // remain from the filtering out process (added).
    mSimulationCollisions.resize(offset + added);

    size_t sizeAfter = mSimulationCollisions.size();
    if (sizeBefore != sizeAfter)
    {
        std::cout << "Filtered out: " << sizeBefore - sizeAfter << " Collisions.\n";
    }
}

bool CollisionManager::isSharingCollisionGroups(
        const std::vector<int>& groupIdsA, const std::vector<int>& groupIdsB)
{
    size_t indexA = 0;
    size_t indexB = 0;
    while (indexA < groupIdsA.size() && indexB < groupIdsB.size())
    {
        if (groupIdsA[indexA] == groupIdsB[indexB])
        {
            return true;
        }
        else if (groupIdsA[indexA] < groupIdsB[indexB])
        {
            ++indexA;
        }
        else
        {
            ++indexB;
        }
    }
    return false;
}
