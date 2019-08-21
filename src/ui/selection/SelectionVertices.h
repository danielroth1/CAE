#ifndef SELECTIONVERTICES_H
#define SELECTIONVERTICES_H

#include "Selection.h"

class SelectionRectangle;
class VertexCollection;

// Manages the selection of vertices
class SelectionVertices : public Selection
{
public:
    SelectionVertices();

    virtual ~SelectionVertices() override;

    void clear() override;

    // Adds the vertices that are inside the 3D
    // view frustum to the vertex collection.
    virtual void updateSelectionByRectangle(
            const std::shared_ptr<SceneLeafData>& leafData,
            ViewFrustum* viewFrustum,
            SelectionRectangle& rectangle) override;

    // TODO: implement this method
    // Adds the vertices that are within a certain
    // range of a at x, y casted ray.
    virtual void updateSelectionByRay(
            const std::shared_ptr<SceneLeafData>& leafData,
            ViewFrustum* viewFrustum,
            int x,
            int y) override;

    // VertexCollection delegated methods
public:
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

private:
    void updateSelectedVertices(
            const std::shared_ptr<SceneLeafData>& leafData,
            ViewFrustum* viewFrustum);

    // TODO: implement this
    // updates selected vertex groups
//    void updateSelectedGroups(SceneLeafData* leafData, ViewFrustum* viewFrustum);

    std::unique_ptr<VertexCollection> mVertexCollection;
};

#endif // SELECTIONVERTICES_H
