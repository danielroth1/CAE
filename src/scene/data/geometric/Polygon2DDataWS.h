#ifndef POLYGONDATAWS_H
#define POLYGONDATAWS_H

#include "Polygon2DData.h"



class Polygon2DDataWS : public Polygon2DData
{
public:
    Polygon2DDataWS(const Faces& faces, ID nVertices);

    Polygon2DDataWS(const Polygon2DTopology& topology);

    virtual ~Polygon2DDataWS() override;

    // Polygon2DData interface
public:
    virtual BSWSVectors::Type getPositionType() override;
};

#endif // POLYGONDATAWS_H
