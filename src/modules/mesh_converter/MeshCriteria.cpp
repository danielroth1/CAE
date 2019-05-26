#include "MeshCriteria.h"


MeshCriteria::MeshCriteria(
        double facetAngle,
        double facetSize,
        double facetDistance,
        double cellSize,
        double cellRadiusEdgeRatio,
        bool edgesAsFeatures,
        double minFeatureEdgeAngleDeg)
    : mFacetAngle(facetAngle)
    , mFacetSize(facetSize)
    , mFaceDistance(facetDistance)
    , mCellSize(cellSize)
    , mCellRadiusEdgeRatio(cellRadiusEdgeRatio)
    , mEdgesAsFeatures(edgesAsFeatures)
    , mMinFeatureEdgeAngleDeg(minFeatureEdgeAngleDeg)
{

}

double MeshCriteria::getFacetAngle() const
{
    return mFacetAngle;
}

void MeshCriteria::setFacetAngle(double facetAngle)
{
    mFacetAngle = facetAngle;
}

double MeshCriteria::getFacetSize() const
{
    return mFacetSize;
}

void MeshCriteria::setFacetSize(double facetSize)
{
    mFacetSize = facetSize;
}

double MeshCriteria::getFaceDistance() const
{
    return mFaceDistance;
}

void MeshCriteria::setFaceDistance(double faceDistance)
{
    mFaceDistance = faceDistance;
}

double MeshCriteria::getCellSize() const
{
    return mCellSize;
}

void MeshCriteria::setCellSize(double cellSize)
{
    mCellSize = cellSize;
}

double MeshCriteria::getCellRadiusEdgeRatio() const
{
    return mCellRadiusEdgeRatio;
}

void MeshCriteria::setCellRadiusEdgeRatio(double cellRadiusEdgeRatio)
{
    mCellRadiusEdgeRatio = cellRadiusEdgeRatio;
}

bool MeshCriteria::isEdgesAsFeatures() const
{
    return mEdgesAsFeatures;
}

void MeshCriteria::setEdgesAsFeatures(bool edgesAsFeatures)
{
    mEdgesAsFeatures = edgesAsFeatures;
}

double MeshCriteria::getMinFeatureEdgeAngleDeg() const
{
    return mMinFeatureEdgeAngleDeg;
}

void MeshCriteria::setMinFeatureEdgeAngleDeg(double minFeatureEdgeAngleDeg)
{
    mMinFeatureEdgeAngleDeg = minFeatureEdgeAngleDeg;
}
