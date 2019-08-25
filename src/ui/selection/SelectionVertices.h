#ifndef SELECTIONVERTICES_H
#define SELECTIONVERTICES_H

#include "Selection.h"

#include <scene/VertexCollection.h>

class SelectionRectangle;
class VertexCollection;

// Manages the selection of vertices
class SelectionVertices : public Selection
{
public:
    SelectionVertices();

    virtual ~SelectionVertices() override;

    // Adds the vertices that are inside the 3D view frustum to the given
    // VertexCollection vcOut.
    void calculateSelectionByRectangle(
            const std::shared_ptr<SceneLeafData>& leafData,
            ViewFrustum* viewFrustum,
            SelectionRectangle& rectangle,
            VertexCollection& vcOut) const;

    // TODO: implement this method
    // Adds the vertices that are within a certain range of a at x, y casted
    // ray to the given VertexCollection vcOut.
    void calculateSelectionByRay(
            const std::shared_ptr<SceneLeafData>& leafData,
            ViewFrustum* viewFrustum,
            int x,
            int y,
            VertexCollection& vcOut) const;

    void updateSelectedVertices(const VertexCollection& vc);

    // Selection interface
public:
    void clear() override;

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

    VertexCollection mVertexCollection;
};

#endif // SELECTIONVERTICES_H
