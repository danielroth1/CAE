#include "Polygon3DTopology.h"
#include "TopologyFactory.h"

#include <iostream>
#include <map>
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

    return createPolygon3DTopology(faces, outerFaces, cells, nVertices);
}

std::shared_ptr<Polygon3DTopology> TopologyFactory::createPolygon3DTopologyWithoutOuter(
        const Faces& faces, const Cells& cells, ID nVertices)
{
    return createPolygon3DTopology(faces, Faces(), cells, nVertices);
}

std::shared_ptr<Polygon3DTopology> TopologyFactory::createPolygon3DTopology(
        const Faces& faces, const Faces& outerFaces, const Cells& cells, ID nVertices)
{
    Faces facesTemp = faces;
    Faces outerFacesTemp = outerFaces;
    Cells cellsTemp = cells;

    // The topology is fixed after creation by calling AbstractPolygon::fixTopology().
    // This can be inefficient, so it would make sense to fix the data before
    // creating the AbstractPolygon. The part that fixes the topology can be done
    // here. It is still necessary to actually remove the vertices in the
    // AbstractPolygon. Uncommenting this method call would result in an inconsistent
    // state and would likely result in a crash.
//    fixTopology(nVertices, facesTemp, outerFacesTemp, cellsTemp);

    return std::make_shared<Polygon3DTopology>(faces, outerFaces, cells, nVertices);
}

std::shared_ptr<Polygon2DTopology> TopologyFactory::createPolygon2DTopology(
        const Faces& faces, ID nVertices)
{
    return std::make_shared<Polygon2DTopology>(faces, nVertices);
}

void TopologyFactory::fixTopology(
        ID nVertices,
        Faces& faces,
        Faces& outerFaces,
        Cells& cells)
{
    std::set<unsigned int> referencedByCells;

    for (Cell& c : cells)
    {
        for (size_t i = 0; i < 4; ++i)
        {
            referencedByCells.insert(c[i]);
        }
    }

    std::vector<unsigned int> notReferencedByCells;
    for (unsigned int i = 0; i < nVertices; ++i)
    {
        if (referencedByCells.find(i) == referencedByCells.end())
        {
            notReferencedByCells.push_back(i);
        }
    }

    if (!notReferencedByCells.empty())
    {
        std::cout << "Fixed topology by removing the following vertices: ";
        for (unsigned int i : notReferencedByCells)
        {
            std::cout << i << " ";
        }
        std::cout << "\n";
    }

    removeVertices(nVertices, notReferencedByCells, faces, outerFaces, cells);
}

void TopologyFactory::removeVertices(
        ID nVertices,
        std::vector<unsigned int>& vertexIds,
        Faces& faces,
        Faces& outerFaces,
        Cells& cells)
{
    if (vertexIds.empty())
        return;

    std::sort(vertexIds.begin(), vertexIds.end());

    //
    // original: 0 1 2 3 4 5 6 7 8 9
    // removed: 3 7
    // new:      0 1 2 3 4 5 6 7
    // oldToNew: 0 1 2 0 3 4 5 0 6 7

    std::vector<unsigned int> oldToNew;
    oldToNew.resize(nVertices);

    unsigned int removedIt = 0;
    for (unsigned int i = 0; i < nVertices; ++i)
    {
        if (removedIt < nVertices && vertexIds[removedIt] == i)
        {
            ++removedIt;
            oldToNew[i] = 0;
        }
        else
        {
            oldToNew[i] = i - removedIt;
        }
    }

    // adapt vectors

    for (Face& f : faces)
    {
        for (size_t i = 0; i < 3; ++i)
        {
            f[i] = oldToNew[f[i]];
        }
    }

    for (Face& f : outerFaces)
    {
        for (size_t i = 0; i < 3; ++i)
        {
            f[i] = oldToNew[f[i]];
        }
    }

    for (Cell& c : cells)
    {
        for (size_t i = 0; i < 4; ++i)
        {
            c[i] = oldToNew[c[i]];
        }
    }
}

