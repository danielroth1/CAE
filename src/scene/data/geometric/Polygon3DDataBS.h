#ifndef SHAREDPOLYGON3DDATABS_H
#define SHAREDPOLYGON3DDATABS_H

#include "Polygon3DData.h"



class Polygon3DDataBS : public Polygon3DData
{
public:
    Polygon3DDataBS(
            const Faces& faces,
            const Faces& outerFaces,
            const Cells& cells,
            const Vectors& positionsBS,
            const Vectors& outerVertexNormalsBS,
            const Vectors& outerFaceNormalsBS);

    Polygon3DDataBS(
            const Polygon3DTopology& topology,
            const Vectors& positionsBS,
            const Vectors& outerVertexNormalsBS,
            const Vectors& outerFaceNormalsBS);

    virtual ~Polygon3DDataBS();

    Vectors& getPositionsBS();
    void setPositionsBS(const Vectors& positionsBS);

    Vectors& getOuterVertexNormalsBS();
    void setOuterVertexNormalsBS(const Vectors& outerVertexNormalsBS);

    Vectors& getOuterFaceNormalsBS();
    void setOuterFaceNormalsBS(const Vectors& outerFaceNormalsBS);

private:

    Vectors mPositionsBS;
    Vectors mOuterVertexNormalsBS;
    Vectors mOuterFaceNormalsBS;
};

#endif // SHAREDPOLYGON3DDATABS_H
