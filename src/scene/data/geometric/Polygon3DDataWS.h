#ifndef POLYGON3DDATAWS_H
#define POLYGON3DDATAWS_H

#include "Polygon3DData.h"


class Polygon3DDataWS : public Polygon3DData
{
public:

    Polygon3DDataWS(const Polygon3DTopology& topology);

    virtual ~Polygon3DDataWS();
};

#endif // POLYGON3DDATAWS_H
