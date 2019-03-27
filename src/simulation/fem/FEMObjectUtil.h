#ifndef FEMOBJECTUTIL_H
#define FEMOBJECTUTIL_H

// Includes
#include "data_structures/DataStructures.h"

// Forward Declarations
class FEMObject;

class FEMObjectUtil
{
public:

    static FEMObjectUtil* instance();

    void actGravity(FEMObject* so, double gravity);

    void actForceOnHighestVertex(FEMObject* so, Eigen::Vector force);

    size_t getTopVertex(FEMObject* so);
    size_t getBottomVertex(FEMObject* so);

private:
    FEMObjectUtil();

    ~FEMObjectUtil();

    static FEMObjectUtil* m_instance;
};

#endif // FEMOBJECTUTIL_H
