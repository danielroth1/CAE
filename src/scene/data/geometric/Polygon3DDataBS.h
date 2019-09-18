#ifndef SHAREDPOLYGON3DDATABS_H
#define SHAREDPOLYGON3DDATABS_H

#include "Polygon3DData.h"



class Polygon3DDataBS : public Polygon3DData
{
public:

    Polygon3DDataBS(
            const std::shared_ptr<Polygon3DTopology>& topology,
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

    // PolygonData interface
public:
    virtual void removeVector(ID index);
    virtual void removeVectors(std::vector<ID>& indices);

private:

    Vectors mPositionsBS;
    Vectors mOuterVertexNormalsBS;
    Vectors mOuterFaceNormalsBS;

};

#endif // SHAREDPOLYGON3DDATABS_H
