#ifndef POLYGON2DTOPOLOGY_H
#define POLYGON2DTOPOLOGY_H

#include <data_structures/DataStructures.h>


class Polygon2DTopology
{
public:
    Polygon2DTopology();

    Polygon2DTopology(
            const Faces& faces,
            const Edges& edges);

    void setFaces(const Faces& faces);
    void setEdges(const Edges& edges);

    Faces& getFaces();
    Edges& getEdges();

private:
    Faces mFaces;
    Edges mEdges;
};

#endif // POLYGON2DTOPOLOGY_H
