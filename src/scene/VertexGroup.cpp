#include "VertexGroup.h"
#include "scene_graph/SceneLeafData.h"
#include "VertexCollection.h"

using namespace Eigen;

VertexGroup::VertexGroup(ID id)
    : mId(id)
{
    mVertexCollection = new VertexCollection();
}

VertexGroup::VertexGroup(ID id, const DataVectorsMap& dvm)
    : mId(id)
{
    mVertexCollection = new VertexCollection(dvm);
}

VertexGroup::~VertexGroup()
{
    if (mVertexCollection)
        delete mVertexCollection;
}

ID VertexGroup::getId() const
{
    return mId;
}

bool VertexGroup::operator<(const VertexGroup &vg) const
{
    return mId < vg.getId();
}

void VertexGroup::addVertex(SceneLeafData* leafData, ID vertexId)
{
    mVertexCollection->addVertex(leafData, vertexId);
}

void VertexGroup::removeVertex(SceneLeafData* leafData, ID vertexId)
{
    mVertexCollection->removeVertex(leafData, vertexId);
}

const VertexCollection*VertexGroup::getVertexCollection() const
{
    return mVertexCollection;
}

const DataVectorsMap& VertexGroup::getDataVectorsMap() const
{
    return mVertexCollection->getDataVectorsMap();
}
