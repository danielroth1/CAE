#include "Polygon.h"

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

void Polygon::updatePositions()
{
    mPositionData.update();
}

BSWSVectors::Type Polygon::getPositionType()
{
    return mPositionData.getType();
}

Edges Polygon::calculateEdges(const Faces& faces)
{
    Edges duplicatedEdges;
    for (const Face& f : faces)
    {
        if (f[0] < f[1] &&
            f[0] < f[2])
        {
            // 9 is smallest
            // (0, 1), (0, 2)
            duplicatedEdges.push_back({f[0], f[1]});
            duplicatedEdges.push_back({f[0], f[2]});
            if (f[1] < f[2])
            {
                // (1, 2)
                duplicatedEdges.push_back({f[1], f[2]});
            }
            else
            {
                // (2, 1)
                duplicatedEdges.push_back({f[2], f[1]});
            }
        }
        else if (f[1] < f[0] &&
                 f[1] < f[2])
        {
             // 1 is smallest
            // (1, 0), (1, 2)
            duplicatedEdges.push_back({f[1], f[0]});
            duplicatedEdges.push_back({f[1], f[2]});
            if (f[0] < f[2])
            {
                // (0, 2)
                duplicatedEdges.push_back({f[0], f[2]});
            }
            else
            {
                // (2, 0)
                duplicatedEdges.push_back({f[2], f[0]});
            }
        }
        else
        {
            // 2 is smallest
            // (2, 0), (2, 1)
            duplicatedEdges.push_back({f[2], f[0]});
            duplicatedEdges.push_back({f[2], f[1]});
            if (f[0] < f[1])
            {
                // (0, 1)
                duplicatedEdges.push_back({f[0], f[1]});
            }
            else
            {
                // (1, 0)
                duplicatedEdges.push_back({f[1], f[0]});
            }
        }
    }

    // all edges with vertex indices (i0, i1) with i0 < i1 are gathered
    // in edges.
    std::map<unsigned int, std::vector<unsigned int>> edgesMap;
    for (const Edge& e : duplicatedEdges)
    {
        auto it = edgesMap.find(e[0]);
        if (it != edgesMap.end())
        {
            std::vector<unsigned int>& edgesEntry = it->second;
            auto it2 = std::find(edgesEntry.begin(), edgesEntry.end(), e[1]);
            if (it2 == edgesEntry.end())
            {
                edgesEntry.push_back(e[1]);
            }
        }
        else
        {
            std::vector<unsigned int> edgesEntry;
            edgesEntry.push_back(e[1]);
            edgesMap[e[0]] = edgesEntry;
        }
    }

    std::vector<Edge> edges;
    for (const std::pair<unsigned int, std::vector<unsigned int>> entry : edgesMap)
    {
        for (unsigned int i2 : entry.second)
        {
            edges.push_back({entry.first, i2});
        }
    }

    return edges;
}

Polygon::~Polygon()
{

}
