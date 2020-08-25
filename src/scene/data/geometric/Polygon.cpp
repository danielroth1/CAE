#include "Polygon.h"
#include "PolygonTopology.h"

#include <iostream>
#include <map>
#include <set>
#include <stack>

#include <data_structures/VectorOperations.h>

Polygon::Polygon()
{

}

Polygon::Polygon(
        const Vectors& positionsWS)
    : mPositionData(positionsWS)
{
}

Polygon::Polygon(
        Vectors* positionsBS,
        const Affine3d& transform)
    : mPositionData(positionsBS, transform)
{
}

void Polygon::initWorldSpace(const Vectors& positionsWS)
{
    mPositionData.initializeFromWorldSpace(positionsWS);
}

void Polygon::initBodySpace(Vectors* positionsBS, const Affine3d& transform)
{
    mPositionData.initializeFromBodySpace(positionsBS, transform);
}

void Polygon::update(bool updateFaceNormals, bool updateVertexNormals, bool notifyListeners)
{
    mPositionData.update();

    GeometricData::update(updateFaceNormals, updateVertexNormals, notifyListeners);
}

void Polygon::removeVertex(ID index)
{
    mPositionData.removePosition(index);
    if (!mNormals.empty())
        VectorOperations::removeVector(mNormals, index);
}

void Polygon::removeVertices(std::vector<ID>& indices)
{
    mPositionData.removePositions(indices);
    if (!mNormals.empty())
        VectorOperations::removeVectors(mNormals, indices.begin(), indices.end());
}

bool Polygon::castRay(
        const Vector3d& origin,
        const Vector3d& normal,
        size_t& triangleIdOut,
        Vector2d& baryOut,
        double& distanceOut)
{
    PolygonTopology& topology = getTopology();
    distanceOut = std::numeric_limits<double>::max();

    double intersects = false;
    Eigen::Vector2d baryTemp;
    double distanceTemp;
    for (size_t i = 0; i < topology.getFacesIndices().size(); ++i)
    {
        if (castRay(i, origin, normal, baryTemp, distanceTemp))
        {
            if (distanceTemp < distanceOut)
            {
                distanceOut = distanceTemp;
                baryOut = baryTemp;
                triangleIdOut = i;
            }

            intersects = true;
        }
    }

    return intersects;
}

bool Polygon::castRay(
        size_t triangleId,
        const Vector3d& origin,
        const Vector3d& normal,
        Vector2d& baryOut,
        double& distanceOut)
{
    const Face& face = getTopology().getFacesIndices()[triangleId];
    const Eigen::Vector3d& p0 = getPosition(face[0]);
    const Eigen::Vector3d& p1 = getPosition(face[1]);
    const Eigen::Vector3d& p2 = getPosition(face[2]);

    Eigen::Vector3d e1(p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]);
    Eigen::Vector3d e2(p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2]);
    Eigen::Vector3d t(origin[0] - p0[0], origin[1] - p0[1], origin[2] - p0[2]);

    Eigen::Vector3d p = normal.cross(e2);
    Eigen::Vector3d q = t.cross(e1);

    double d1 = p.dot(e1);
    if (fabs(d1) < 10e-7)
        return false;

    double f = 1.0f / d1;
    baryOut(0) = f * (p.dot(t));

    if (baryOut(0) < 0 || baryOut(0) > 1.0)
        return false;

    baryOut(1) = f * (q.dot(normal));
    if (baryOut(1) < 0.0 || baryOut(1) > 1.0 || (baryOut(0) + baryOut(1)) > 1.0)
        return false;

    distanceOut = f * (q.dot(e2));

    return true;
}

GeometricData::Type Polygon::getType() const
{
    return Type::POLYGON;
}

Vectors& Polygon::getPositions()
{
    return mPositionData.getPositions();
}

Affine3d& Polygon::getTransform()
{
    return mPositionData.getTransform();
}

Vectors& Polygon::getPositionsBS()
{
    return mPositionData.getPositionsBS();
}

Vector& Polygon::getPositionBS(ID index)
{
    return mPositionData.getPositionBS(index);
}

Vector Polygon::getCenter() const
{
    return mPositionData.getCenter();
}

void Polygon::setTransform(const Affine3d& transform)
{
    mPositionData.setTransform(transform);
}

Vector Polygon::calculateCenterVertex()
{
    return mPositionData.calculateCenterVertex();
}

Vector Polygon::calculateCenterOfMass(const std::vector<double>& masses)
{
    return mPositionData.calculateCenterOfMass(masses);
}

Vector& Polygon::getPosition(size_t index)
{
    return mPositionData.getPosition(index);
}

void Polygon::setPosition(size_t index, const Vector& position)
{
    mPositionData.setPosition(index, position);
}

size_t Polygon::getSize()
{
    return mPositionData.getSize();
}

void Polygon::translate(const Vector& position)
{
    mPositionData.translate(position);
}

void Polygon::transform(const Affine3d& transform)
{
    mPositionData.transform(transform);
}

BSWSVectors::Type Polygon::getPositionType()
{
    return mPositionData.getType();
}

const ID* Polygon::getRelevantFaces(const TopologyFeature& feature, size_t& count)
{
    switch(feature.getType())
    {
    case TopologyFeature::Type::VERTEX:
    {
        const TopologyVertex& vertex = static_cast<const TopologyVertex&>(feature);

        count = vertex.getFaceIds().size();
        return vertex.getFaceIds().data();
    }
    case TopologyFeature::Type::EDGE:
    {
        const TopologyEdge& edge = static_cast<const TopologyEdge&>(feature);

        count = edge.getFaceIds().size();
        return edge.getFaceIds().data();
    }
    case TopologyFeature::Type::FACE:
    {
        const TopologyFace& face = static_cast<const TopologyFace&>(feature);
        count = 1;
        return &face.getIDRef();
    }
    case TopologyFeature::Type::CELL:
    {
        count = 0;
        break;
    }
    }
    return nullptr;
}

Polygon::~Polygon()
{

}

bool Polygon::isInside(
        const TopologyFeature& feature,
        Vector point,
        PolygonTopology& topology,
        BSWSVectors& faceNormals)
{
    size_t count;
    const ID* relevantFaces = getRelevantFaces(feature, count);

    if (count == 0)
        return false;

    for (size_t i = 0; i < count; ++i)
    {
        if (!isInside(relevantFaces[i], point, topology, faceNormals))
            return false;
    }

    return true;
}

bool Polygon::isInside(
        const TopologyFeature& feature,
        Vector source,
        double distance,
        Vector target,
        PolygonTopology& topology,
        BSWSVectors& faceNormals)
{
    // measure the distance of each visited face and check if its vertices are within distance
    // if not, visit the adjacent faces. Make sure to not visit faces twice by using a
    // set.

    std::set<ID> visited; // visited face ids
    std::stack<ID> tbv; // to be visited face ids

    size_t count;
    const ID* relevantFaces = getRelevantFaces(feature, count);

    if (count == 0)
        return false;

    for (size_t i = 0; i < count; ++i)
    {
        tbv.push(relevantFaces[i]);
        visited.insert(relevantFaces[i]);
    }

    while (!tbv.empty())
    {
        ID currentFaceId = tbv.top();
        TopologyFace& currentFace = topology.getFace(currentFaceId);
        tbv.pop();
        visited.insert(currentFaceId);

        // add adjacent faces that weren't visited before to tbv stack
        for (ID id : currentFace.getAdjacentFaces())
        {
            if (visited.find(id) == visited.end() &&
                isWithinDistance(topology.getFace(id), source, distance))
            {
                tbv.push(id);
            }
        }

        if (!isInside(currentFaceId, target, topology, faceNormals))
            return false;
    }

    return true;
}

bool Polygon::isInside(
        ID faceId,
        const Vector& point,
        PolygonTopology& topology,
        BSWSVectors& faceNormals)
{
    // this vertexId is w.r.t. outerPositionsIds?
    // mPositionsData.getPosition(mOuterPositionIds[vertexId])
    ID vertexId = topology.getFace(faceId).getVertexIds()[0];
    return faceNormals.getVector(faceId)
            .dot(point - mPositionData.getPosition(vertexId)) < 0;
}

bool Polygon::isWithinDistance(
        TopologyFace& face,
        const Eigen::Vector& point,
        double distance)
{
    for (size_t i = 0; i < 3; ++i)
    {
        double actualdistance = (mPositionData.getPosition(face.getVertexIds()[i]) - point).norm();
        if (actualdistance < distance)
            return true;
    }
    return false;
}
