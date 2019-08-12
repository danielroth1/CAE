#include "Polygon3DTopology.h"
#include "TopologyFactory.h"

#include <set>

TopologyFactory::TopologyFactory()
{

}

std::shared_ptr<Polygon3DTopology> TopologyFactory::createPolygon3DTopology(
        const Cells& cells, ID nVertices)
{
    Faces faces;

    // assemble faces
    auto sortedFace = [](std::array<unsigned int, 3> f)
    {
        std::sort(f.begin(), f.end());
        return f;
    };

    std::set<std::array<unsigned int, 3>> addedSortedFacets;
    auto addFaceIfNotContains = [&sortedFace, &faces, &addedSortedFacets](
            std::array<unsigned int, 3> f)
    {
        std::array<unsigned int, 3> sortedF = sortedFace(f);
        if (addedSortedFacets.find(sortedF) == addedSortedFacets.end())
        {
            addedSortedFacets.insert(sortedF);
            faces.push_back(f);
        }
    };

    for (const Cell& c : cells)
    {
        addFaceIfNotContains({c[0], c[1], c[2]});
        addFaceIfNotContains({c[0], c[1], c[3]});
        addFaceIfNotContains({c[0], c[2], c[3]});
        addFaceIfNotContains({c[1], c[2], c[3]});
    }

    return createPolygon3DTopologyWithOuter(faces, cells, nVertices);
}

std::shared_ptr<Polygon3DTopology> TopologyFactory::createPolygon3DTopologyWithOuter(
        const Faces& faces, const Cells& cells, ID nVertices)
{
    std::shared_ptr<Polygon3DTopology> topo =
            createPolygon3DTopologyWithoutOuter(faces, cells, nVertices);

    Faces outerFaces;
    for (TopologyFace& face : topo->getFaces())
    {
        if (face.getCellIds().size() <= 1)
            outerFaces.push_back(topo->getFacesIndices()[face.getID()]);
    }

    return std::make_shared<Polygon3DTopology>(faces, outerFaces, cells, nVertices);
}

std::shared_ptr<Polygon3DTopology> TopologyFactory::createPolygon3DTopologyWithoutOuter(
        const Faces& faces, const Cells& cells, ID nVertices)
{
    return std::make_shared<Polygon3DTopology>(faces, Faces(), cells, nVertices);
}

std::shared_ptr<Polygon3DTopology> TopologyFactory::createPolygon3DTopology(
        const Faces& faces, const Faces& outerFaces, const Cells& cells, ID nVertices)
{
    return std::make_shared<Polygon3DTopology>(faces, outerFaces, cells, nVertices);
}

std::shared_ptr<Polygon2DTopology> TopologyFactory::createPolygon2DTopology(
        const Faces& faces, ID nVertices)
{
    return std::make_shared<Polygon2DTopology>(faces, nVertices);
}
