#include "SelectionSceneData.h"
#include "SelectionSceneDataModel.h"

#include <scene/model/PolygonRenderModel.h>
#include <scene/model/RenderModelVisitor.h>

#include <rendering/Appearance.h>
#include <rendering/Appearances.h>
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

    mSelectionAppearance =
            std::make_shared<Appearances>(
                Appearance::createAppearanceFromColor({0.0f, 1.0f, 0.0f, 1.0f}));
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

    std::vector<std::shared_ptr<SceneData>> notSelectedButColored;
    for (const std::shared_ptr<SceneData>& sd : mSelectedColored)
    {
        if (mSelectionSceneData.getSceneData().find(sd) ==
            mSelectionSceneData.getSceneData().end())
        {
            notSelectedButColored.push_back(sd);
        }
    }

    if (notSelectedButColored.size() > 0)
        std::cout << notSelectedButColored.size() << "\n";

    class ResetOrignalAppearanceVisitor : public RenderModelVisitor
    {
    public:
        ResetOrignalAppearanceVisitor(
                    const std::shared_ptr<Appearances>& _appearance)
            : appearance(_appearance)
        {

        }

        virtual void visit(PolygonRenderModel& model)
        {
            model.setRenderedAppearances(model.getAppearances());
        }

        virtual void visit(LinearForceRenderModel& /*model*/)
        {

        }

        const std::shared_ptr<Appearances>& appearance;
    } vReset(mSelectionAppearance);

    for (const std::shared_ptr<SceneData>& sd : notSelectedButColored)
    {
        if (sd->isLeafData())
        {
            std::shared_ptr<SceneLeafData> ld =
                    std::static_pointer_cast<SceneLeafData>(sd);
            if (ld->getRenderModel())
                ld->getRenderModel()->accept(vReset);
        }
    }

    // set back all appearances that are not selected anymore but were previously
    // ...

    class RMV : public RenderModelVisitor
    {
    public:
        RMV(const std::shared_ptr<Appearances>& _appearance)
            : appearance(_appearance)
        {

        }

        virtual void visit(PolygonRenderModel& model)
        {
            model.setRenderedAppearances(appearance);
        }

        virtual void visit(LinearForceRenderModel& /*model*/)
        {

        }

        const std::shared_ptr<Appearances>& appearance;
    } vSet(mSelectionAppearance);

    for (const std::shared_ptr<SceneData>& sd : mSelectionSceneData.getSceneData())
    {
        if (sd->isLeafData())
        {
            std::shared_ptr<SceneLeafData> sceneData =
                    std::static_pointer_cast<SceneLeafData>(sd);

            if (sceneData->getRenderModel())
            {
                sceneData->getRenderModel()->accept(vSet);
            }

            // Render points. Not used at the moment to avoid confusion between
            // selected vertices and selected scene nodes (selected vertices
            // are also indicated by rendered points).
//            GeometricData* gd = sceneData->getGeometricDataRaw();
//            for (ID i = 0; i < gd->getSize(); ++i)
//            {
//                Vector& v = gd->getPosition(i);
//                points->push_back(Vector3f(
//                                     static_cast<float>(v(0)),
//                                     static_cast<float>(v(1)),
//                                     static_cast<float>(v(2))));
//            }
        }
    }

    mSelectedColored = mSelectionSceneData.getSceneData();
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
