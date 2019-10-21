#ifndef POLYGON3DDATA_H
#define POLYGON3DDATA_H

#include "data_structures/DataStructures.h"

#include "PolygonData.h"

class Polygon3DTopology;


class Polygon3DData : public PolygonData
{
public:

    Polygon3DData(const std::shared_ptr<Polygon3DTopology>& topology);

    virtual ~Polygon3DData() override;

    const std::shared_ptr<Polygon3DTopology>& getTopology() const;

    // PolygonData interface
public:
    virtual Polygon::DimensionType getDimensionType() const override;

    // Removes a vector at the given index.
    virtual void removeVector(ID index) override;

    // Removes the vectors at the given indices.
    virtual void removeVectors(std::vector<ID>& indices) override;

private:

    std::shared_ptr<Polygon3DTopology> mTopology;

};

#endif // POLYGON3DDATA_H
