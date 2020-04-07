#include "Polygon3DTopology.h"

#include <iostream>
#include <map>
#include <set>

#include <data_structures/VectorOperations.h>


Polygon3DTopology::Polygon3DTopology(
        const Faces& faces,
        const Faces& outerFaces, // you can not call this constructor with already 2d outer faces
        const Cells& cells,
        ID nVertices)
    : PolygonTopology (faces, nVertices)
    , mOuterFacesIndices3D(outerFaces)
    , mOuterVertexIds(calculateOuterVertexIDs(mOuterFacesIndices3D))
    , mCellIds(cells)
{
    init();

    std::cout << "V = " << nVertices << ", " <<
                 "E = " << mEdges.size() << ", " <<
                 "F = " << mFaces.size() << ", " <<
                 "T = " << mCells.size() << "\n";
    std::cout << "V - E + F - T = " <<
                 nVertices - mEdges.size() + mFaces.size() - mCells.size() << "\n";

    // Check if all vertices are referenced by cells.
    std::set<unsigned int> verticesReferencedInCells;
    for (TopologyCell& tc : mCells)
    {
        for (size_t i = 0; i < 4; ++i)
        {
            verticesReferencedInCells.insert(tc.getVertexIds()[i]);
        }
    }

    bool requiresFix = false;
    if (verticesReferencedInCells.size() != nVertices)
    {
        std::cout << nVertices - verticesReferencedInCells.size()
                  << " dangling vertices. Potential not possible"
                     " factorization.\n";
        requiresFix = true;
    }

    // Check if no cell points to two identical vertices.
    for (size_t i = 0; i < mCellIds.size(); ++i)
    {
        Cell& c = mCellIds[i];
        if (c[0] == c[1] || c[0] == c[2] || c[0] == c[3] ||
            c[1] == c[2] || c[1] == c[3] ||
            c[2] == c[3])
        {
            std::cout << "Detected degenerated cell that points to two"
                         " identical vertices.\n";
            requiresFix = true;
        }
    }

    if (requiresFix)
        std::cout << "The topology requires to be fixed before it can be "
                     "used with the Finite Element Method.\n";
}

void Polygon3DTopology::removeVertices(std::vector<ID>& vertexIds)
{
    if (vertexIds.empty())
        return;

    size_t nVerticesAfter = mVertices.size() - vertexIds.size();

    std::vector<ID> oldToNew = createOldToNewMapping(vertexIds, mVertices.size());

    // 1.) remove all faces / cells that reference the to be removed vertices
    // from:
    //  mOuterFacesIndices3D
    //  mFacesIndices
    //  mCellIds
    // 2.) adapt the indices in the remaining faces / cells (w.r.t. the removed vertices)
    // change indices in:
    //  mOuterFacesIndices3D
    //  mFacesIndices
    //  mCellIds
    // 3.) reinit everything with the adapted data
    //  mOuterVertexIds
    //  mOuterTopology

    // 1.)
    VectorOperations::removeArrayIfContains<unsigned int, 3>(
                vertexIds.begin(),
                vertexIds.end(),
                mOuterFacesIndices3D);
    VectorOperations::removeArrayIfContains<unsigned int, 3>(
                vertexIds.begin(),
                vertexIds.end(),
                mFacesIndices);
    VectorOperations::removeArrayIfContains<unsigned int, 4>(
                vertexIds.begin(),
                vertexIds.end(),
                mCellIds);

    // 2.)
    // mOuterFacesIndices3D
    for (size_t fId = 0; fId < mOuterFacesIndices3D.size(); ++fId)
    {
        for (size_t i = 0; i < 3; ++i)
        {
            mOuterFacesIndices3D[fId][i] = static_cast<unsigned int>(
                        oldToNew[mOuterFacesIndices3D[fId][i]]);
        }
    }

    // mFacesIndices
    for (size_t fId = 0; fId < mFacesIndices.size(); ++fId)
    {
        for (size_t i = 0; i < 3; ++i)
        {
            mFacesIndices[fId][i] = static_cast<unsigned int>(
                        oldToNew[mFacesIndices[fId][i]]);
        }
    }

    // mCellIds
    for (size_t cId = 0; cId < mCellIds.size(); ++cId)
    {
        for (size_t i = 0; i < 4; ++i)
        {
            mCellIds[cId][i] = static_cast<unsigned int>(
                        oldToNew[mCellIds[cId][i]]);
        }
    }

    // 3.)
    mOuterVertexIds = calculateOuterVertexIDs(mOuterFacesIndices3D);

    PolygonTopology::init(mFacesIndices, nVerticesAfter);
    init();
}

std::vector<ID> Polygon3DTopology::retrieveNotReferencedByCells() const
{
    std::set<ID> referenced;

    for (const TopologyCell& c : mCells)
    {
        for (size_t i = 0; i < 4; ++i)
        {
            referenced.insert(c.getVertexIds()[i]);
        }
    }

    std::vector<ID> allVertices;
    allVertices.reserve(mVertices.size());
    for (size_t i = 0; i < mVertices.size(); ++i)
    {
        allVertices.push_back(i);
    }

    return VectorOperations::sub(allVertices, referenced);
}

Cells& Polygon3DTopology::getCellIds()
{
    return mCellIds;
}

std::vector<TopologyCell>& Polygon3DTopology::getCells()
{
    return mCells;
}

Polygon2DTopology& Polygon3DTopology::getOuterTopology()
{
    return mOuterTopology;
}

const Polygon2DTopology& Polygon3DTopology::getOuterTopology() const
{
    return mOuterTopology;
}

std::vector<unsigned int>& Polygon3DTopology::getOuterVertexIds()
{
    return mOuterVertexIds;
}

void Polygon3DTopology::setOuterVertexIds(
        const std::vector<unsigned int>& outerVertexIds)
{
    mOuterVertexIds = outerVertexIds;
}

std::vector<unsigned int>& Polygon3DTopology::getOuterFaceIds()
{
    return mOuterFaceIds;
}

void Polygon3DTopology::setOuterFaceIds(const std::vector<unsigned int>& outerFaceIds)
{
    mOuterFaceIds = outerFaceIds;
}

std::vector<TopologyEdge>& Polygon3DTopology::getOuterEdges()
{
    return mOuterTopology.getEdges();
}

const std::vector<TopologyEdge>& Polygon3DTopology::getOuterEdges() const
{
    return mOuterTopology.getEdges();
}

std::vector<TopologyFace>& Polygon3DTopology::getOuterFaces()
{
    return mOuterTopology.getFaces();
}

const std::vector<TopologyFace>& Polygon3DTopology::getOuterFaces() const
{
    return mOuterTopology.getFaces();
}

Faces& Polygon3DTopology::getOuterFacesIndices()
{
    return mOuterTopology.getFacesIndices();
}

const Faces& Polygon3DTopology::getOuterFacesIndices() const
{
    return mOuterTopology.getFacesIndices();
}

Faces& Polygon3DTopology::getOuterFacesIndices3D()
{
    return mOuterFacesIndices3D;
}

const Faces& Polygon3DTopology::getOuterFacesIndices3D() const
{
    return mOuterFacesIndices3D;
}

Edges Polygon3DTopology::retrieveOuterEdges() const
{
    return mOuterTopology.retrieveEdges();
}

void Polygon3DTopology::init()
{

    PolygonTopology::init(mFacesIndices, mVertices.size());

    // Cells: add vertex ids
    mCells.clear();
    mCells.reserve(mCellIds.size());
    for (size_t cId = 0; cId < mCellIds.size(); ++cId)
    {
        mCells.push_back(TopologyCell(cId, mVertices.size() + mEdges.size() + mFaces.size() + cId));
        TopologyCell& c = mCells[cId];
        c.getVertexIds() = mCellIds[cId];

        // Vertex: add cell ids
        for (size_t i = 0; i < 4; ++i)
        {
            TopologyVertex& v = mVertices[c.getVertexIds()[i]];
            v.getCellIds().push_back(cId);
        }
    }

    // Fill the adjacent cell indices of each face
    // To do so, iterate over all vertices and insert each cell id to all
    // neighbored faces (if not already done).

    auto sortedFace = [](std::array<unsigned int, 3> f)
    {
        std::sort(f.begin(), f.end());
        return f;
    };

    auto sortedEdge = [](std::array<unsigned int, 2> e)
    {
        std::sort(e.begin(), e.end());
        return e;
    };

    // create triangle map: maps triangle vertex ids to triangle ids
    // Triangles are stored in their array representation with sorted ascending
    // vertex ids. This makes them better findable when iterating over cells.
    std::map<std::array<unsigned int, 3>, size_t> triangleMap;
    for (size_t fId = 0; fId < mFaces.size(); ++fId)
    {
        auto it = triangleMap.find(sortedFace(mFaces[fId].getVertexIds()));
        if (it != triangleMap.end())
            std::cout << "Warning: found doubled face.\n";
        triangleMap[sortedFace(mFaces[fId].getVertexIds())] = fId;
    }

    // Cells: add triangle ids
    // For each cell, extract the corresponding triangles from the triangle map
    // Add a triangle id for each permutation.
    for (size_t cId = 0; cId < mCells.size(); ++cId)
    {
        TopologyCell& tc = mCells[cId];
        Cell& c = mCells[cId].getVertexIds();
        tc.getFaceIds().push_back(static_cast<unsigned int>(
                                      triangleMap[sortedFace({c[0], c[1], c[2]})]));
        tc.getFaceIds().push_back(static_cast<unsigned int>(
                                      triangleMap[sortedFace({c[0], c[1], c[3]})]));
        tc.getFaceIds().push_back(static_cast<unsigned int>(
                                      triangleMap[sortedFace({c[0], c[2], c[3]})]));
        tc.getFaceIds().push_back(static_cast<unsigned int>(
                                      triangleMap[sortedFace({c[1], c[2], c[3]})]));
    }

    // mOuterFaceIds
    mOuterFaceIds.clear();
    mOuterFaceIds.reserve(mOuterFacesIndices3D.size());
    for (Face& f : mOuterFacesIndices3D)
    {
        if (triangleMap.find(sortedFace(f)) == triangleMap.end())
        {
            std::cout << "Outer face has no cell.\n";
        }
        unsigned int index = static_cast<unsigned int>(
                    triangleMap[sortedFace(f)]);
        mOuterFaceIds.push_back(index);
    }

    // Cells: add edge ids
    // Finally add edges by adding all edges of the 4 faces.
    // Should be 6 per tetrahedron.
    for (size_t cId = 0; cId < mCells.size(); ++cId)
    {
        TopologyCell& tc = mCells[cId];
        std::set<unsigned int> edgeIds;
        for (unsigned int fId : tc.getFaceIds())
        {
            for (unsigned int eId : mFaces[fId].getEdgeIds())
            {
                edgeIds.insert(eId);
            }
        }
        if (edgeIds.size() > 6)
        {
            std::cout << "Warning: More than 6 edges found for tetrahedron " << cId << "\n";
        }

        if (edgeIds.size() < 6)
        {
            std::cerr << "Fatal: Tetrahedron has less than 6 edges.\n";
            continue;
        }

        size_t it = 0;
        for (auto edgeId : edgeIds)
        {
            if (it == 6)
                break;
            tc.getEdgeIds()[it] = edgeId;
            ++it;
        }
    }

    // Edge: add cell ids
    // Faces: add cell ids
    for (size_t cId = 0; cId < mCells.size(); ++cId)
    {
        TopologyCell& tc = mCells[cId];

        for (unsigned int eId : tc.getEdgeIds())
        {
            mEdges[eId].getCellIds().push_back(cId);
        }
        for (unsigned int fId : tc.getFaceIds())
        {
            mFaces[fId].getCellIds().push_back(cId);
        }
    }

    mOuterTopology = Polygon2DTopology(
                transformTo2DIndices(mOuterFacesIndices3D, mOuterVertexIds),
                mOuterVertexIds.size());

    // mOuterEdgeIds
    {
        // edgeMap: edge -> id
        std::map<Edge, size_t> edgeMap;
        for (size_t eId = 0; eId < mEdges.size(); ++eId)
        {
            auto it = edgeMap.find(sortedEdge(mEdges[eId].getVertexIds()));
            if (it != edgeMap.end())
                std::cout << "Warning: found doubled edge.\n";
            edgeMap[sortedEdge(mEdges[eId].getVertexIds())] = eId;
        }

        // what are the outer edges?
        mOuterEdgeIds.clear();
        mOuterEdgeIds.reserve(mOuterTopology.getEdges().size());
        for (TopologyEdge& te : mOuterTopology.getEdges())
        {
            Edge e = {mOuterVertexIds[te.getVertexIds()[0]],
                     mOuterVertexIds[te.getVertexIds()[1]]};

            unsigned int index = static_cast<unsigned int>(
                        edgeMap[sortedEdge(e)]);
            mOuterEdgeIds.push_back(index);
        }
    }

    for (size_t i = 0; i < mOuterTopology.getFaces().size(); ++i)
    {
        TopologyFace& f3d = mFaces[to3DFaceIndex(i)];

        if (f3d.getCellIds().size() == 0)
        {
            std::cout << "Warning: Outer face has zero cells.\n";
        }
        if (f3d.getCellIds().size() > 1)
        {
            std::cout << "Warning: Outer face has more than one cell: cells = "
                      << f3d.getCellIds().size() << ".\n";
        }
    }
}

std::vector<unsigned int> Polygon3DTopology::calculateOuterVertexIDs(const Faces& faces)
{
    std::vector<unsigned int> outerPositionIds;

    std::set<unsigned int> idsSet;
    for (const Face& face : faces)
    {
        idsSet.insert(face[0]);
        idsSet.insert(face[1]);
        idsSet.insert(face[2]);
    }

    outerPositionIds.reserve(idsSet.size());
    for (unsigned int id : idsSet)
    {
        outerPositionIds.push_back(id);
    }

//    std::move(idsSet.begin(), idsSet.end(), outerPositionIds.begin());
    std::sort(outerPositionIds.begin(), outerPositionIds.end(), std::less<unsigned int>());

    return outerPositionIds;
}

Faces Polygon3DTopology::transformTo2DIndices(
        const Faces& faces,
        const std::vector<unsigned int>& outerVertexIds) const
{
    std::map<unsigned int, unsigned int> reverseMap;

    for (unsigned int i = 0; i < outerVertexIds.size(); ++i)
    {
        reverseMap[outerVertexIds[i]] = i;
    }

    Faces facesOut;
    facesOut.resize(faces.size());
    for (size_t i = 0; i < faces.size(); ++i)
    {
        for (size_t j = 0; j < 3; ++j)
        {
            facesOut[i][j] = reverseMap[faces[i][j]];
        }
    }

    return facesOut;
}
