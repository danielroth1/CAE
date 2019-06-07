#include "SelectionVerticesModel.h"
#include "SelectionVertices.h"

#include <rendering/RenderMaterial.h>
#include <rendering/Renderer.h>
#include <rendering/object/RenderPoints.h>
#include <rendering/object/RenderScreenRectangle.h>
#include <scene/VertexCollection.h>
#include <scene/data/GeometricDataVisitor.h>
#include <scene/data/geometric/GeometricPoint.h>
#include <scene/data/geometric/Polygon2D.h>
#include <scene/data/geometric/Polygon3D.h>

using namespace Eigen;

SelectionVerticesModel::SelectionVerticesModel(
        SelectionVertices& selection)
    : mSelection(selection)
{
    mRenderPoints = std::make_shared<RenderPoints>();
    mRenderPoints->setRenderMaterial(
                RenderMaterial::createFromColor({0.0f, 1.0f, 0.0f, 1.0f}));
}

SelectionVerticesModel::~SelectionVerticesModel()
{
}

void SelectionVerticesModel::updatePoints()
{
    auto points = mRenderPoints->getPoints().lock();
    points->clear();
    if (!mSelection.isActive())
        return;

    // TODO: way too many dependencies
    const DataVectorsMap& map = mSelection.getSelectedVertexCollection()->getDataVectorsMap();
    for (auto it = map.begin(); it != map.end(); ++it)
    {
        for (ID id : it->second)
        {
            class CollectPointsVisitor : public GeometricDataVisitor
            {
            public:
                CollectPointsVisitor(Vectorfs& _points, ID _id)
                    : points(_points)
                    , id(_id)
                { }

                virtual void visit(Polygon2D& polygon2D)
                {
                    Vector& v = polygon2D.getPosition(id);
                    points.push_back(Vectorf(static_cast<float>(v(0)),
                                              static_cast<float>(v(1)),
                                              static_cast<float>(v(2))));
                }
                virtual void visit(Polygon3D& polygon3D)
                {
                    Vector& v = polygon3D.getPosition(id);
                    points.push_back(Vectorf(static_cast<float>(v(0)),
                                              static_cast<float>(v(1)),
                                              static_cast<float>(v(2))));
                }
                virtual void visit(GeometricPoint& gp)
                {
                    Vector& v = gp.getPosition();
                    points.push_back(Vectorf(static_cast<float>(v(0)),
                                             static_cast<float>(v(1)),
                                             static_cast<float>(v(2))));
                }

                Vectorfs& points;
                ID id;
            };

            CollectPointsVisitor visitor(*points, id);
            it->first->getGeometricDataRaw()->accept(visitor);
//            Vector& v = it->first->getGeometricData()->getPosition(id);
//            mPoints.push_back(Vectorf(static_cast<float>(v(0)),
//                                      static_cast<float>(v(1)),
//                                      static_cast<float>(v(2))));
        }
    }
//    mPoints.clear();
//    mPoints.reserve(points.size());
//    for (const Vector& p : points)
//    {
//        mPoints.push_back(Vectorf(p(0), p(1), p(2)));
    //    }
}

void SelectionVerticesModel::reset()
{

}

void SelectionVerticesModel::update()
{
    updatePoints();
}

void SelectionVerticesModel::revalidate()
{
    updatePoints();
}

void SelectionVerticesModel::accept(RenderModelVisitor& v)
{

}

void SelectionVerticesModel::addToRenderer(Renderer* renderer)
{
    renderer->addRenderObject(mRenderPoints);
}

void SelectionVerticesModel::removeFromRenderer(Renderer* renderer)
{
    renderer->removeRenderObject(mRenderPoints);
}

void SelectionVerticesModel::setVisible(bool visible)
{
    mRenderPoints->setVisible(visible);
    RenderModel::setVisible(visible);
}
