#ifndef VERTEXGROUP_H
#define VERTEXGROUP_H

#include "data_structures/DataStructures.h"
#include <set>
#include <vector>
#include <map>

class SceneLeafData;
class VertexCollection;

typedef std::map<std::shared_ptr<SceneLeafData>, std::vector<ID>> DataVectorsMap;

// A VertexCollection that has an ID.
class VertexGroup
{
public:
    VertexGroup(ID id);
    VertexGroup(ID id, const DataVectorsMap& dvm);
    ~VertexGroup();

    ID getId() const;

    // Operators
    bool operator<(VertexGroup const& vg) const;

    // Delegated Methods from Vertex Collection:
    // Adder
    void addVertex(const std::shared_ptr<SceneLeafData>& leafData, ID vertexId);
    void removeVertex(const std::shared_ptr<SceneLeafData>& leafData, ID vertexId);
    // Getter
    const VertexCollection* getVertexCollection() const;
    const DataVectorsMap& getDataVectorsMap() const;

private:
    ID mId;

    VertexCollection* mVertexCollection;
};

#endif // VERTEXGROUP_H
