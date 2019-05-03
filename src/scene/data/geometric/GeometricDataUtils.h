#ifndef GEOMETRICDATAUTILS_H
#define GEOMETRICDATAUTILS_H

#include "data_structures/DataStructures.h"

#include <data_structures/BoundingBox.h>

// Consider using the functions from ModelUtils instead.
class GeometricDataUtils
{
public:
    static Vectors calculateNormals(
            const Vectors& positions,
            const Faces& faces);

    static void updateBoundingBox(
            Vectors& positions,
            BoundingBox& boundingBox);

protected:
    GeometricDataUtils();
};

#endif // GEOMETRICDATAUTILS_H
