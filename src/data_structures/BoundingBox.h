#ifndef BOUNDINGBOX_H
#define BOUNDINGBOX_H

#include "data_structures/DataStructures.h"

using namespace Eigen;

class BoundingBox
{
public:
    BoundingBox();

    Vector& min();
    Vector& max();
    Vector& mid();
    Vector& size();


private:
    Vector m_min;
    Vector m_max;
    Vector m_mid;
    Vector m_size;
};

#endif // BOUNDINGBOX_H
