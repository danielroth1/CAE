#include "PositionData.h"

using namespace Eigen;

PositionData::PositionData()
{

}

PositionData::PositionData(
        const Vectors& positionsWS)
    : mPositions(positionsWS)
{
}

PositionData::PositionData(
        Vectors* positionsBS,
        const Affine3d& transform)
    : mPositions(positionsBS, transform)
{
}

PositionData::PositionData(const PositionData& posData)
    : mPositions(posData.mPositions)
{
    mCenter = posData.mCenter;
}

// go to BSWSVectors
void PositionData::update()
{
    mPositions.update();
}

BSWSVectors::Type PositionData::getType()
{
    return mPositions.getType();
}

size_t PositionData::getSize()
{
    return mPositions.getSize();
}

Vectors& PositionData::getPositions()
{
    return mPositions.getVectors();
}

Vector& PositionData::getPosition(ID index)
{
    return mPositions.getVector(index);
}

Affine3d& PositionData::getTransform()
{
    return mPositions.getTransform();
}

Vectors& PositionData::getPositionsBS()
{
    return mPositions.getVectorsBS();
}

Vector& PositionData::getPositionBS(ID index)
{
    return mPositions.getVectorBS(index);
}

Vector PositionData::getCenter() const
{
    return mCenter;
}

void PositionData::setTransform(const Affine3d& transform)
{
    mPositions.setTransform(transform);
}

Vector PositionData::calculateCenterVertex()
{
    if (mPositions.getType() == BSWSVectors::Type::BODY_SPACE)
        updateWorldSpace();
    Vector center = Vector::Zero();
    for (size_t i = 0; i < mPositions.getSize(); ++i)
    {
        center += mPositions.getVector(i);
    }
    center /= mPositions.getSize();
    return center;
}

Vector PositionData::calculateCenterOfMass(
        const std::vector<double>& masses)
{
    if (mPositions.getType() == BSWSVectors::Type::BODY_SPACE)
        updateWorldSpace();
    Vector center = Vector::Zero();
    double massesSum = 0.0;
    for (size_t i = 0; i < mPositions.getSize(); ++i)
    {
        // this is not correct!
        center += masses[i] * mPositions.getVector(i);
        massesSum += masses[i];
    }
    center /= massesSum;
    return center;
}

// go to BSWSVectors without center ( = (0, 0, 0) )
// setting the center should be done only in SharedPolygonData
void PositionData::changeRepresentationToBS(
        Vectors* vectorsBS,
        const Eigen::Affine3d& transform)
{
    mPositions.changeRepresentationToBS(vectorsBS, transform);
}

void PositionData::changeRepresentationToWS()
{
    mPositions.changeRepresentationToWS();
}

void PositionData::translate(const Vector& t)
{
    mPositions.translate(t);
}

void PositionData::transform(const Affine3d& transform)
{
    mPositions.transform(transform);
}

void PositionData::initializeFromWorldSpace(Vectors positionsWS)
{
    mPositions.initializeFromWorldSpace(positionsWS);
    mCenter.setZero();
}

// go to BSWSVectors
void PositionData::initializeFromBodySpace(
        Vectors* positoinsBS,
        const Affine3d& transform)
{
    mPositions.initializeFromBodySpace(positoinsBS, transform);
    mCenter.setZero();
}

void PositionData::moveCenterTo(const Vector& center)
{
    moveCenter(-mCenter);
    moveCenter(center);
}

// this operation should not be possible in Polygon
// only in shared data and should only be done on initialization
// because it affects all Polygons that point on this data.
void PositionData::moveCenter(const Vector& deltaCenter)
{
    Vectors& positionsBS = mPositions.getVectorsBS();
    Vectors& positionsWS = mPositions.getVectors();
    for (size_t i = 0; i < positionsBS.size(); ++i)
    {
        positionsBS[i] = positionsWS[i] - deltaCenter;
    }
    mPositions.getTransform().translate(deltaCenter);

    mCenter += deltaCenter;
}

void PositionData::updateWorldSpace()
{
    mPositions.updateWorldSpace();
}

