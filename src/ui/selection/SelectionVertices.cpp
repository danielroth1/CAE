#include "SelectionRectangle.h"
#include "SelectionVertices.h"

#include <data_structures/DataStructures.h>
#include <scene/data/GeometricDataVisitor.h>
#include <scene/data/geometric/GeometricPoint.h>
#include <scene/data/geometric/Polygon2D.h>
#include <scene/data/geometric/Polygon3D.h>
#include <scene/scene_graph/SceneLeafData.h>
#include <ui/KeyManager.h>

#include <iostream>

using namespace Eigen;

SelectionVertices::SelectionVertices()
{
}

SelectionVertices::~SelectionVertices()
{
}

void SelectionVertices::calculateSelectionByRectangle(
        const std::shared_ptr<SceneLeafData>& leafData,
        ViewFrustum* viewFrustum,
        SelectionRectangle& rectangle,
        VertexCollection& vcOut) const
{
    std::vector<ID> vectors;

    // retrieve vectors of geometric data
    class Visitor : public GeometricDataVisitor
    {
    public:
        Visitor(const SelectionVertices& _s,
                std::vector<ID>& _vectors,
                SelectionRectangle& _sr,
                ViewFrustum* _vf)
            : s(_s)
            , vectors(_vectors)
            , sr(_sr)
            , vf(_vf)
        {

        }

        void test(Vector& v, ID id)
        {
            if (sr.testVertex(v, vf))
                vectors.push_back(id);
        }

        void visit(Polygon2D& polygon2D)
        {
            for (ID id = 0; id < polygon2D.getSize(); ++id)
                test(polygon2D.getPosition(id), id);
        }

        void visit(Polygon3D& polygon3D)
        {
            for (ID id = 0; id < polygon3D.getSize(); ++id)
                test(polygon3D.getPosition(id), id);
        }

        void visit(GeometricPoint& gp)
        {
            test(gp.getPosition(), 0);
        }

        const SelectionVertices& s;
        std::vector<ID>& vectors;
        SelectionRectangle& sr;
        ViewFrustum* vf;
    } visitor(*this, vectors, rectangle, viewFrustum);

    leafData->getGeometricDataRaw()->accept(visitor);

    if (!vectors.empty())
        vcOut.addVertices(leafData, vectors);
}

void SelectionVertices::calculateSelectionByRay(
        const std::shared_ptr<SceneLeafData>& leafData,
        ViewFrustum* viewFrustum,
        int x,
        int y,
        VertexCollection& vcOut) const
{
    SelectionRectangle sr;
    int range = 1;
    sr.setRectangle(x - range, y - range,
                    x + range, y + range);
    calculateSelectionByRectangle(leafData, viewFrustum, sr, vcOut);
}

void SelectionVertices::updateSelectedVertices(const VertexCollection& vc)
{
    if (KeyManager::instance()->isShiftDown())
    {
        // add element
        mVertexCollection.addVertices(vc.getDataVectorsMap());
    }
    else if (KeyManager::instance()->isCtrlDown())
    {
        // remove element
        mVertexCollection.removeVertices(vc.getDataVectorsMap());
    }
    else
    {
        // clear elements, then add
        clear();
        mVertexCollection = vc;
    }
}

void SelectionVertices::clear()
{
    mVertexCollection.clear();
}

void SelectionVertices::addVertex(
        const std::shared_ptr<SceneLeafData>& leafData, ID vertexID)
{
    mVertexCollection.addVertex(leafData, vertexID);
}

void SelectionVertices::addVertices(
        const std::shared_ptr<SceneLeafData>& leafData, std::vector<ID>& vectors)
{
    mVertexCollection.addVertices(leafData, vectors);
}

void SelectionVertices::removeVertex(
        const std::shared_ptr<SceneLeafData>& leafData, ID vertexID)
{
    mVertexCollection.removeVertex(leafData, vertexID);
}

void SelectionVertices::removeVertices(
        const std::shared_ptr<SceneLeafData>& leafData)
{
    mVertexCollection.removeVertices(leafData);
}

const DataVectorsMap& SelectionVertices::getDataVectorsMap() const
{
    return mVertexCollection.getDataVectorsMap();
}
