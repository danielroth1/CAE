#include "MeshConverterUIControl.h"
#include "MeshConverterUIForm.h"

#include <ApplicationControl.h>
#include <QWidget>

#include <scene/scene_graph/SceneData.h>
#include <scene/scene_graph/SceneDataVisitor.h>
#include <scene/scene_graph/SceneLeafData.h>

#include <scene/data/GeometricData.h>
#include <scene/data/GeometricDataVisitor.h>

#include <scene/data/geometric/GeometricPoint.h>
#include <scene/data/geometric/Polygon2D.h>
#include <scene/data/geometric/Polygon2DTopology.h>
#include <scene/data/geometric/Polygon3D.h>
#include <scene/data/geometric/Polygon3DTopology.h>

#include <ui/UIControl.h>

#include <ui/selection/SelectionControl.h>

#include <modules/mesh_converter/MeshConverterControl.h>
#include <modules/mesh_converter/MeshConverterModule.h>
#include <modules/mesh_converter/MeshCriteria.h>


MeshConverterUIControl::MeshConverterUIControl(MeshConverterModule* module, ApplicationControl* ac)
    : mAc(ac)
    , mModule(module)
{
}

MeshConverterUIControl::~MeshConverterUIControl()
{

}

void MeshConverterUIControl::init(QWidget* parent)
{
    mWidget = new MeshConverterUIForm(this, parent);

    mAc->getUIControl()->getSelectionControl()->addListener(this);
}

QWidget* MeshConverterUIControl::getWidget()
{
    return mWidget;
}

void MeshConverterUIControl::onConvertButtonClicked()
{
    MeshCriteria meshCriteria(
                mWidget->getFacetAngle(),
                mWidget->getFacetSize(),
                mWidget->getFacetDistance(),
                mWidget->getCellSize(),
                mWidget->getCellRadiusEdgeRatio(),
                mWidget->isSharpFeaturesEnabled(),
                mWidget->getMinFeatureEdgeAngleDeg());

    mModule->getControl()->convert(meshCriteria);
}

void MeshConverterUIControl::onRevertButtonClicked()
{
    mModule->getControl()->revert();
}

void MeshConverterUIControl::onSceneNodeSelected(const std::shared_ptr<SceneData>& sd)
{
    processSceneData(sd);
}

void MeshConverterUIControl::onSelectedSceneNodesChanged(
        const std::set<std::shared_ptr<SceneData>>& sd)
{
    if (sd.empty())
    {
        onSelectionCleared();
    }
    else if (sd.size() == 1)
    {
        processSceneData(*sd.begin());
    }
}

void MeshConverterUIControl::onSelectedVerticesChanged(
        const std::map<std::shared_ptr<SceneLeafData>, std::vector<ID> >& /*sv*/)
{

}

void MeshConverterUIControl::onSelectionCleared()
{
    mWidget->updateGeometricData(0, 0, 0);
}

void MeshConverterUIControl::processSceneData(const std::shared_ptr<SceneData>& sd)
{
    class Visitor : public SceneDataVisitor
    {
    public:
        Visitor(MeshConverterUIControl& _mcc)
            : mcc(_mcc)
        {
        }

        virtual void visit(SceneData* /*sceneData*/)
        {

        }

        virtual void visit(SceneLeafData* sceneData)
        {
            mcc.processGeometricData(sceneData->getGeometricDataRaw());
        }

        MeshConverterUIControl& mcc;
    } visitor(*this);

    sd->accept(&visitor);
}

void MeshConverterUIControl::processGeometricData(GeometricData* gd)
{
    class Visitor : public GeometricDataVisitor
    {
    public:
        Visitor(MeshConverterUIControl& _mcc)
            : mcc(_mcc)
        {
        }

        virtual void visit(Polygon2D& polygon2D)
        {
            mcc.mWidget->updateGeometricData(
                        static_cast<int>(polygon2D.getPositions().size()),
                        static_cast<int>(polygon2D.getTopology().getFaces().size()),
                        0);
        }

        virtual void visit(Polygon3D& polygon3D)
        {
            mcc.mWidget->updateGeometricData(
                        static_cast<int>(polygon3D.getPositions().size()),
                        static_cast<int>(polygon3D.getTopology3D().getFaces().size()),
                        static_cast<int>(polygon3D.getTopology3D().getCells().size()));
        }

        virtual void visit(GeometricPoint& /*point*/)
        {
            mcc.mWidget->updateGeometricData(1, 0, 0);
        }

        MeshConverterUIControl& mcc;
    } visitor(*this);
    gd->accept(visitor);

}

