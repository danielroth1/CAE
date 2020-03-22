#ifndef POLYGON2DTOPOLOGY_H
#define POLYGON2DTOPOLOGY_H

#include "PolygonTopology.h"

#include <data_structures/DataStructures.h>


class Polygon2DTopology : public PolygonTopology
{
public:

    Polygon2DTopology();

    Polygon2DTopology(const Faces& faces, ID nVertices);

};

#endif // POLYGON2DTOPOLOGY_H
