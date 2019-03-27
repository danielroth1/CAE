#include "PositionDataRigid.h"

GeometricDataRigid::GeometricDataRigid(const Vectors& positionsBS)
    : mPositionsBS(positionsBS)
    , mPositionsWS(positionsBS)
{
    mTransform.setIdentity();
}

Vector& GeometricDataRigid::getPosition(size_t index)
{
    return mPositionsWS[index];
}

size_t GeometricDataRigid::getSize()
{
    return mPositionsBS.size();
}

Vector& GeometricDataRigid::getPositionBS(size_t index)
{
    return mPositionsBS[index];
}

Affine3d& GeometricDataRigid::getTransform()
{
    return mTransform;
}

void GeometricDataRigid::updateWorldSpacePositions()
{
    for (size_t i = 0; i < getSize(); ++i)
    {
        mPositionsWS[i] = mTransform * mPositionsBS[i];
    }
}

void GeometricDataRigid::setCenter(Vector center)
{
    for (size_t i = 0; i < getSize(); ++i)
    {
        mPositionsBS[i] -= center;
    }
    mTransform.translate(-center);
}

