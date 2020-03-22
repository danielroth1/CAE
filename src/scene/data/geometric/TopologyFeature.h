#ifndef TOPOLOGYFEATURE_H
#define TOPOLOGYFEATURE_H

#include <data_structures/DataStructures.h>

class TopologyFeature
{
public:
    TopologyFeature(ID id);
    virtual ~TopologyFeature();

    enum class Type
    {
        VERTEX, EDGE, FACE, CELL
    };

    ID getID() const;
    const ID& getIDRef() const;

    void setID(ID id);

    virtual Type getType() const = 0;

private:
    ID mId;
};

#endif // TOPOLOGYFEATURE_H
