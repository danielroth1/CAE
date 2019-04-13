#include "SelectionRectangle.h"
#include "SelectionVertices.h"

#include <data_structures/DataStructures.h>
#include <scene/data/GeometricDataVisitor.h>
#include <scene/VertexCollection.h>
#include <scene/data/geometric/GeometricPoint.h>
#include <scene/data/geometric/Polygon2D.h>
#include <scene/data/geometric/Polygon3D.h>
#include <scene/scene_graph/SceneLeafData.h>

#include <iostream>

using namespace Eigen;

SelectionVertices::SelectionVertices()
{
    mVertexCollection = std::make_unique<VertexCollection>();
}

SelectionVertices::~SelectionVertices()
{
    //    for (auto it : mVertexGroups)
    //        delete it;
}

VertexCollection* SelectionVertices::getSelectedVertexCollection()
{
    return mVertexCollection.get();
}

void SelectionVertices::clear()
{
    mVertexCollection->clear();
}

void SelectionVertices::updateSelectionByRectangle(
        const std::shared_ptr<SceneLeafData>& leafData,
        ViewFrustum* viewFrustum,
        SelectionRectangle& rectangle)
{
    std::vector<ID> vectors;

    // retrieve vectors of geometric data
    class Visitor : public GeometricDataVisitor
    {
    public:
        Visitor(SelectionVertices& _s,
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

        SelectionVertices& s;
        std::vector<ID>& vectors;
        SelectionRectangle& sr;
        ViewFrustum* vf;
    } visitor(*this, vectors, rectangle, viewFrustum);

    leafData->getGeometricDataRaw()->accept(visitor);

    if (!vectors.empty())
        mVertexCollection->addVertices(leafData, vectors);
}

void SelectionVertices::updateSelectionByRay(
        const std::shared_ptr<SceneLeafData>& leafData,
        ViewFrustum* viewFrustum,
        int x,
        int y)
{
    SelectionRectangle sr;
    int range = 1;
    sr.setRectangle(x - range, y - range,
                    x + range, y + range);
    updateSelectionByRectangle(leafData, viewFrustum, sr);
}

//void SelectionVertices::updateSelectedGroups(
//        SceneLeafData* leafData, ViewFrustum* viewFrustum)
//{
//    // TODO: implement me
//}
