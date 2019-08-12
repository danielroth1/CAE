#ifndef TOPOLOGYFACTORY_H
#define TOPOLOGYFACTORY_H

#include <data_structures/DataStructures.h>
#include <memory>

class Polygon2DTopology;
class Polygon3DTopology;

class TopologyFactory
{
public:
    TopologyFactory();

    // Creates a Polygon3DTopology with neither faces nor outer faces. Both
    // are created from the cells.
    static std::shared_ptr<Polygon3DTopology> createPolygon3DTopology(
            const Cells& cells,
            ID nVertices);

    // Creates a Polygon3DTopology and also automatically calculates and
    // adds the outer topology. Faces of the outer topology are those that have
    // fewer than 2 neighbored cells.
    static std::shared_ptr<Polygon3DTopology> createPolygon3DTopologyWithOuter(
            const Faces& faces,
            const Cells& cells,
            ID nVertices);

    // Creates a Polygon3DTopology wihtout outer faces. Use this if an outer
    // Polygon is for some reason not required. If unsure, use
    // createPolygon3DTopologyWithOuter instead.
    static std::shared_ptr<Polygon3DTopology> createPolygon3DTopologyWithoutOuter(
            const Faces& faces,
            const Cells& cells,
            ID nVertices);

    // Creates a regular Polygon3DTopology.
    static std::shared_ptr<Polygon3DTopology> createPolygon3DTopology(
            const Faces& faces,
            const Faces& outerFaces,
            const Cells& cells,
            ID nVertices);

    // Creates a regular Polygon2DTopology.
    static std::shared_ptr<Polygon2DTopology> createPolygon2DTopology(
            const Faces& faces, ID nVertices);
};

#endif // TOPOLOGYFACTORY_H
