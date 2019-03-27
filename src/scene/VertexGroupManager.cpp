#include "VertexGroupManager.h"
#include "scene/VertexGroup.h"

VertexGroupManager::VertexGroupManager()
{
    mIdCounter = 0;
}

VertexGroup *VertexGroupManager::createVertexGroup()
{
    VertexGroup* group = new VertexGroup(mIdCounter++);
    mGroupMap[group->getId()] = group;
    return group;
}

VertexGroup *VertexGroupManager::createVertexGroup(const DataVectorsMap& dvm)
{
    VertexGroup* group = new VertexGroup(mIdCounter++, dvm);
    mGroupMap[group->getId()] = group;
    return group;
}

VertexGroup* VertexGroupManager::getVertexGroup(ID id)
{
    return mGroupMap[id];
}

void VertexGroupManager::removeVertexGroup(ID id)
{
    // TODO
    mGroupMap.erase(id);
}
