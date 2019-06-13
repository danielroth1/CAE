#ifndef POLYGONINDEXMAPPING_H
#define POLYGONINDEXMAPPING_H

#include <data_structures/DataStructures.h>

#include <memory>

class VectorIndexMapping;

class PolygonIndexMapping
{
public:
    PolygonIndexMapping(
            const std::shared_ptr<VectorIndexMapping>& vectorIndexMapping,
            const Faces& faces);

    std::shared_ptr<VectorIndexMapping> getVectorIndexMapping() const;
    Faces& getFaces();


private:
    std::shared_ptr<VectorIndexMapping> mVectorIndexMapping;
    Faces mFaces;

};

#endif // POLYGONINDEXMAPPING_H
