#include "VertexCollection.h"
#include <scene/scene_graph/SceneLeafData.h>

using namespace Eigen;

VertexCollection::VertexCollection()
{

}

VertexCollection::VertexCollection(const DataVectorsMap& dvm)
{
    mDataVectorsMap = dvm;
}

void VertexCollection::addVertex(
        const std::shared_ptr<SceneLeafData>& leafData,
        ID vertexID)
{
    DataVectorsMap::iterator it = mDataVectorsMap.find(leafData);
    if (it != mDataVectorsMap.end())
    {
        it->second.push_back(vertexID);
    }
    else
    {
        std::vector<ID> vec;
        vec.push_back(vertexID);
        mDataVectorsMap[leafData] = vec;
    }
}

void VertexCollection::addVertices(
        const std::shared_ptr<SceneLeafData>& leafData,
        std::vector<ID>& vectors)
{
    DataVectorsMap::iterator it = mDataVectorsMap.find(leafData);
    if (it != mDataVectorsMap.end())
    {
        it->second.insert(it->second.end(), vectors.begin(), vectors.end());
    }
    else
    {
        mDataVectorsMap[leafData] = vectors;
    }
}

void VertexCollection::removeVertex(
        const std::shared_ptr<SceneLeafData>& leafData,
        ID vertexID)
{
    DataVectorsMap::iterator it = mDataVectorsMap.find(leafData);
    if (it != mDataVectorsMap.end())
    {
        std::remove(it->second.begin(), it->second.end(), vertexID);
    }
}

void VertexCollection::removeVertices(
        const std::shared_ptr<SceneLeafData>& leafData)
{
    mDataVectorsMap.erase(leafData);
}

const DataVectorsMap& VertexCollection::getDataVectorsMap() const
{
    return mDataVectorsMap;
}

void VertexCollection::setDataVectorsMap(DataVectorsMap& dvm)
{
    mDataVectorsMap = dvm;
}

void VertexCollection::clear()
{
    mDataVectorsMap.clear();
}
