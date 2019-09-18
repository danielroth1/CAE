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

void Polygon::update()
{
    mPositionData.update();

    GeometricData::update();
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
