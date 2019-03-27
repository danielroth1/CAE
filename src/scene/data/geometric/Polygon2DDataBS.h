#ifndef POLYGON2DDATABS_H
#define POLYGON2DDATABS_H

#include "BSWSVectors.h"
#include "Polygon2DData.h"

#include <data_structures/DataStructures.h>


class Polygon2DDataBS : public Polygon2DData
{
public:
    Polygon2DDataBS(
            const Faces& faces,
            const Edges& edges,
            const Vectors& positionsBS,
            const Vectors& vertexNormalsBS,
            const Vectors& faceNormalsBS);

    void setPositionsBS(const Vectors& positionsBS);
    void setVertexNormalsBS(const Vectors& vertexNormalsBS);
    void setFaceNormalsBS(const Vectors& faceNormalsBS);

    Vectors& getPositionsBS();
    Vectors& getVertexNormalsBS();
    Vectors& getFaceNormalsBS();

    // Polygon2DData interface
public:
    virtual BSWSVectors::Type getPositionType() override;

private:

    Vectors mPositionsBS;
    Vectors mVertexNormalsBS;
    Vectors mFacesNormalsBS;

};

#endif // Polygon2DDataBS_H
