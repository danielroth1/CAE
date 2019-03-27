#ifndef VERTEXCOLLECTION_H
#define VERTEXCOLLECTION_H

// Includes
#include <data_structures/DataStructures.h>
#include <map>
#include <vector>

class SceneLeafData;

typedef std::map<SceneLeafData*, std::vector<ID>> DataVectorsMap;

class VertexCollection
{
public:
    VertexCollection();
    VertexCollection(const DataVectorsMap& dvm);

    // Adder
    void addVertex(SceneLeafData* leafData, ID vertexID);
    void addVertices(SceneLeafData* leafData, std::vector<ID>& vectors);

    // Remover
    void removeVertex(SceneLeafData* leafData, ID vertexID);
    void removeVertices(SceneLeafData* leafData);

    // Getter
    const DataVectorsMap& getDataVectorsMap() const;

    // Setter
    void setDataVectorsMap(DataVectorsMap& dvm);

    void clear();

private:

    DataVectorsMap mDataVectorsMap;
};

#endif // VERTEXCOLLECTION_H
