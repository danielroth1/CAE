#include "PolygonTopology.h"
#include "TopologyEdge.h"


#include <map>
#include <set>

PolygonTopology::PolygonTopology(const Faces& faces, ID nVertices)
    : mFacesIndices(faces)
{
    buildTopology(faces, nVertices, mVertices, mEdges, mFaces);
}

TopologyVertex& PolygonTopology::getVertex(ID id)
{
    return mVertices[id];
}

TopologyEdge& PolygonTopology::getEdge(ID id)
{
    return mEdges[id];
}

TopologyFace& PolygonTopology::getFace(ID id)
{
    return mFaces[id];
}

std::vector<TopologyVertex>& PolygonTopology::getVertices()
{
    return mVertices;
}

const std::vector<TopologyVertex>& PolygonTopology::getVertices() const
{
    return mVertices;
}

std::vector<TopologyEdge>& PolygonTopology::getEdges()
{
    return mEdges;
}

const std::vector<TopologyEdge>& PolygonTopology::getEdges() const
{
    return mEdges;
}

std::vector<TopologyFace>& PolygonTopology::getFaces()
{
    return mFaces;
}

const std::vector<TopologyFace>& PolygonTopology::getFaces() const
{
    return mFaces;
}

Faces& PolygonTopology::getFacesIndices()
{
    return mFacesIndices;
}

const Faces& PolygonTopology::getFacesIndices() const
{
    return mFacesIndices;
}

Edges PolygonTopology::retrieveEdges() const
{
    Edges edges;
    edges.reserve(mEdges.size());
    for (const TopologyEdge& e : mEdges)
    {
        const std::vector<ID>& vIds = e.getVertexIds();
        edges.push_back({static_cast<unsigned int>(vIds[0]),
                         static_cast<unsigned int>(vIds[1])});
    }
    return edges;
}

Faces PolygonTopology::retrieveFaces() const
{
    Faces faces;
    faces.reserve(mFaces.size());
    for (const TopologyFace& f : mFaces)
    {
        const Face& vIds = f.getVertexIds();
        faces.push_back({static_cast<unsigned int>(vIds[0]),
                         static_cast<unsigned int>(vIds[1]),
                         static_cast<unsigned int>(vIds[2])});
    }
    return faces;
}

std::string PolygonTopology::toString() const
{
    std::stringstream ss;
    ss << "Number of vertices: " << mVertices.size() <<
          "Number of edges: " << mEdges.size() <<
          "Number of faces: " << mFaces.size();
    return ss.str();

}

std::vector<TopologyEdge>
PolygonTopology::calculateEdges(const Faces& faces) const
{
    // First create a vector of all edges of all faces.
    // Insert edges (v1, v2) so that v1 < v2

    std::vector<Edge> duplicatedEdges;
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

    // edgesMap stores the
    // all edges with vertex indices (i0, i1) with i0 < i1 are gathered
    // in edges.
    std::map<ID, std::vector<ID>> edgesMap;
    for (Edge& e : duplicatedEdges)
    {
        auto it = edgesMap.find(e[0]);
        if (it != edgesMap.end())
        {
            std::vector<ID>& edgesEntry = it->second;
            auto it2 = std::find(edgesEntry.begin(), edgesEntry.end(), e[1]);
            if (it2 == edgesEntry.end())
            {
                edgesEntry.push_back(e[1]);
            }
        }
        else
        {
            std::vector<ID> edgesEntry;
            edgesEntry.push_back(e[1]);
            edgesMap[e[0]] = edgesEntry;
        }
    }

    std::vector<TopologyEdge> edges;
    for (const std::pair<unsigned int, std::vector<ID>> entry : edgesMap)
    {
        for (ID i2 : entry.second)
        {
            TopologyEdge e(edges.size());
            e.getVertexIds().push_back(entry.first);
            e.getVertexIds().push_back(i2);
            edges.push_back(e);
        }
    }

    return edges;
}

void PolygonTopology::buildTopology(
        const Faces& faces,
        ID nVertices,
        std::vector<TopologyVertex>& verticesOut,
        std::vector<TopologyEdge>& edgesOut,
        std::vector<TopologyFace>& facesOut) const
{
    // TopologyVertices
    verticesOut.reserve(nVertices);
    for (ID i = 0; i < nVertices; ++i)
    {
        verticesOut.push_back(TopologyVertex(verticesOut.size()));
    }

    // create set of vertices
    std::set<std::pair<ID, ID>> edgesSet;
    for (const Face& f : faces)
    {
        edgesSet.insert(makeSmallerPair(f[0], f[1]));
        edgesSet.insert(makeSmallerPair(f[0], f[2]));
        edgesSet.insert(makeSmallerPair(f[1], f[2]));
    }

    // TopologyEdges
    edgesOut.reserve(edgesSet.size());
    for (const std::pair<ID, ID>& p : edgesSet)
    {
        TopologyEdge te(edgesOut.size());
        // add edge start and end vertex
        te.getVertexIds().push_back(p.first);
        te.getVertexIds().push_back(p.second);

        // add this edge to both its vertices
        verticesOut[p.first].getEdgeIds().push_back(edgesOut.size());
        verticesOut[p.second].getEdgeIds().push_back(edgesOut.size());

        edgesOut.push_back(te);
    }

    // points from edges represented by vertex ids to edge ids
    std::map<std::pair<unsigned int, unsigned int>, unsigned int> edgesMap;
    for (unsigned int i = 0; i < edgesOut.size(); ++i)
    {
        TopologyEdge& p = edgesOut[i];
        edgesMap[std::make_pair(p.getVertexIds()[0], p.getVertexIds()[1])] = i;
    }

    // TopologyFaces
    facesOut.reserve(faces.size());
    for (ID i = 0; i < faces.size(); ++i)
    {
        const Face& f = faces[i];
        TopologyFace tFace(facesOut.size());
        // add surrounding edges
        tFace.getEdgeIds()[0] = edgesMap[makeSmallerPair(f[0], f[1])];
        tFace.getEdgeIds()[1] = edgesMap[makeSmallerPair(f[0], f[2])];
        tFace.getEdgeIds()[2] = edgesMap[makeSmallerPair(f[1], f[2])];

        // add surrounding vertices
        tFace.getVertexIds()[0] = f[0];
        tFace.getVertexIds()[1] = f[1];
        tFace.getVertexIds()[2] = f[2];


        // add this face to all surrounding edges and vertices
        for (ID j = 0; j < 3; ++j)
        {
            verticesOut[tFace.getVertexIds()[j]].getFaceIds().push_back(i);
            edgesOut[tFace.getEdgeIds()[j]].getFaceIds().push_back(i);
        }

        facesOut.push_back(tFace);
    }

    // insert adjacent faces
    for (ID i = 0; i < facesOut.size(); ++i)
    {
        TopologyFace& tFace = facesOut[i];
        for (ID j = 0; j < 3; ++j)
        {
            tFace.getAdjacentFaces().push_back(edgesOut[tFace.getEdgeIds()[j]].getOtherFaceId(i));
        }
    }
}

std::pair<ID, ID> PolygonTopology::makeSmallerPair(ID i1, ID i2) const
{
    if (i1 < i2)
        return std::make_pair(i1, i2);
    return std::make_pair(i2, i1);
}
