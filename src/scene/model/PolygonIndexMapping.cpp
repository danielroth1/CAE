#include "PolygonIndexMapping.h"

PolygonIndexMapping::PolygonIndexMapping(
        const std::shared_ptr<VectorIndexMapping>& vectorIndexMapping,
        const Faces& faces)
    : mVectorIndexMapping(vectorIndexMapping)
    , mFaces(faces)
{

}

std::shared_ptr<VectorIndexMapping> PolygonIndexMapping::getVectorIndexMapping() const
{
    return mVectorIndexMapping;
}

Faces& PolygonIndexMapping::getFaces()
{
    return mFaces;
}
