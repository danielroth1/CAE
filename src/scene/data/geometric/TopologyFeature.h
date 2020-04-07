#ifndef TOPOLOGYFEATURE_H
#define TOPOLOGYFEATURE_H

#include <data_structures/DataStructures.h>

class TopologyFeature
{
public:
    TopologyFeature(ID id, ID geometryId);
    virtual ~TopologyFeature();

    enum class Type
    {
        VERTEX, EDGE, FACE, CELL
    };

    ID getID() const;
    const ID& getIDRef() const;

    void setID(ID id);

    ID getGeometryID() const
    {
        return mGeometryID;
    }

    void setGeometryID(ID geometryID)
    {
        mGeometryID = geometryID;
    }

    virtual Type getType() const = 0;

private:
    ID mId; // Unique id within the feature type (e.g. for all vertices).
    ID mGeometryID; // Unique id within the geometry. Uniquely identies any features between all other features.
};

#endif // TOPOLOGYFEATURE_H
