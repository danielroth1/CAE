#include "Polygon.h"
#include "PolygonTopology.h"

#include <iostream>
#include <map>

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

void Polygon::updatePositions()
{
    mPositionData.update();
}

BSWSVectors::Type Polygon::getPositionType()
{
    return mPositionData.getType();
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
    auto inside = [&](ID faceId) mutable
    {
        ID vertexId = topology.getFace(faceId).getVertexIds()[0];
        Eigen::Vector v1 = faceNormals.getVector(faceId);
        Eigen::Vector v2 = point - mPositionData.getPosition(vertexId);
        if (v1.dot(v2) < 0)
        {
            return true;
        }
        return false;
    };

    switch(feature.getType())
    {
    case TopologyFeature::Type::VERTEX:
    {
        const TopologyVertex& vertex = static_cast<const TopologyVertex&>(feature);

        if (vertex.getFaceIds().empty())
            return false;

        for (ID faceId : vertex.getFaceIds())
        {
            if (!inside(faceId))
                return false;
        }
        break;
    }
    case TopologyFeature::Type::EDGE:
    {
        const TopologyEdge& edge = static_cast<const TopologyEdge&>(feature);

        if (edge.getFaceIds().empty())
            return false;

        for (ID faceId : edge.getFaceIds())
        {
            if (!inside(faceId))
                return false;
        }
        break;
    }
    case TopologyFeature::Type::FACE:
    {
        const TopologyFace& face = static_cast<const TopologyFace&>(feature);
        return inside(face.getID());
    }
    }
    return true;
}
