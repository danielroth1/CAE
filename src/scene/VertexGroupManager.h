#ifndef VERTEXGROUPMANAGER_H
#define VERTEXGROUPMANAGER_H

// Includes
#include "ui/UniqueVertex.h"
#include <map>
#include <set>
#include <vector>

// Forward declarations
class SceneLeafData;
class VertexGroup;

typedef std::map<std::shared_ptr<SceneLeafData>, std::vector<ID>> DataVectorsMap;

class VertexGroupManager
{
public:
    VertexGroupManager();

    // Create a new vertex group
    VertexGroup* createVertexGroup();
    VertexGroup* createVertexGroup(const DataVectorsMap& dvm);

    // Getters
    VertexGroup* getVertexGroup(ID id);

    // Remove vertex group
    void removeVertexGroup(ID id);


private:
    std::map< ID, VertexGroup* > mGroupMap;
    std::map< UniqueVertex, VertexGroup* > mUvGroupMap;
    ID mIdCounter; // vertex grouop id counter
};

#endif // VERTEXGROUPMANAGER_H
