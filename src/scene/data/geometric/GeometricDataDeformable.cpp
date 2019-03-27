#include "PositionDataDeformable.h"

GeometricDataDeformable::GeometricDataDeformable(const Vectors& positions)
    : mPositions(positions)
{

}

Vector& GeometricDataDeformable::getPosition(size_t index)
{
    return mPositions[index];
}

size_t GeometricDataDeformable::getSize()
{
    return mPositions.size();
}
