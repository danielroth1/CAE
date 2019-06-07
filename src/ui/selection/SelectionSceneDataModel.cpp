#include "SelectionSceneData.h"
#include "SelectionSceneDataModel.h"

#include <scene/model/RenderModelVisitor.h>

#include <rendering/RenderMaterial.h>
#include <rendering/Renderer.h>

#include <rendering/object/RenderPoints.h>

#include <scene/scene_graph/SceneData.h>
#include <scene/scene_graph/SGCore.h>
#include <scene/scene_graph/SGTraverserFactory.h>

#include <scene/data/GeometricData.h>

SelectionSceneDataModel::SelectionSceneDataModel(SelectionSceneData& selectionSceneData)
    : mSelectionSceneData(selectionSceneData)
{
    mRenderPoints = std::make_shared<RenderPoints>();
    mRenderPoints->setRenderMaterial(
                RenderMaterial::createFromColor({0.0f, 1.0f, 0.0f, 1.0f}));
}

SelectionSceneDataModel::~SelectionSceneDataModel()
{

}

void SelectionSceneDataModel::reset()
{

}

void SelectionSceneDataModel::update()
{
    auto points = mRenderPoints->getPoints().lock();

    points->clear();
    if (!mSelectionSceneData.isActive())
        return;

    for (const std::shared_ptr<SceneData>& sd : mSelectionSceneData.getSceneData())
    {
        class Visitor : public SceneDataVisitor
        {
        public:
            Visitor(Vectorfs& _points)
                : points(_points)
            {
            }

            virtual void visit(SceneData* /*sceneData*/)
            {

            }

            virtual void visit(SceneLeafData* sceneData)
            {
                GeometricData* gd = sceneData->getGeometricDataRaw();
                for (ID i = 0; i < gd->getSize(); ++i)
                {
                    Vector& v = gd->getPosition(i);
                    points.push_back(Vector3f(
                                         static_cast<float>(v(0)),
                                         static_cast<float>(v(1)),
                                         static_cast<float>(v(2))));
                }
            }

            Vectorfs& points;
        } v(*points);

        sd->accept(&v);
    }
}

void SelectionSceneDataModel::revalidate()
{

}

void SelectionSceneDataModel::accept(RenderModelVisitor& /*v*/)
{

}

void SelectionSceneDataModel::addToRenderer(Renderer* renderer)
{
    renderer->addRenderObject(mRenderPoints);
}

void SelectionSceneDataModel::removeFromRenderer(Renderer* renderer)
{
    renderer->removeRenderObject(mRenderPoints);
}

void SelectionSceneDataModel::setVisible(bool visible)
{
    mRenderPoints->setVisible(visible);
    RenderModel::setVisible(visible);
}
