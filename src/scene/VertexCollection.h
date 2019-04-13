#ifndef VERTEXCOLLECTION_H
#define VERTEXCOLLECTION_H

// Includes
#include <data_structures/DataStructures.h>
#include <map>
#include <vector>

class SceneLeafData;

typedef std::map<std::shared_ptr<SceneLeafData>, std::vector<ID>> DataVectorsMap;

class VertexCollection
{
public:
    VertexCollection();
    VertexCollection(const DataVectorsMap& dvm);

    // Adder
    void addVertex(
            const std::shared_ptr<SceneLeafData>& leafData,
            ID vertexID);
    void addVertices(
            const std::shared_ptr<SceneLeafData>& leafData,
            std::vector<ID>& vectors);

    // Remover
    void removeVertex(
            const std::shared_ptr<SceneLeafData>& leafData,
            ID vertexID);
    void removeVertices(
            const std::shared_ptr<SceneLeafData>& leafData);

    // Getter
    const DataVectorsMap& getDataVectorsMap() const;

    // Setter
    void setDataVectorsMap(DataVectorsMap& dvm);

    void clear();

private:

    DataVectorsMap mDataVectorsMap;
};

#endif // VERTEXCOLLECTION_H
