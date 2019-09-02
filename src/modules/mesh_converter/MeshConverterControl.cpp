#include "MeshConverter.h"
#include "MeshConverterControl.h"
#include "MeshCriteria.h"

#include <ApplicationControl.h>

#include <ui/UIControl.h>

#include <ui/selection/SelectionControl.h>

#include <scene/data/GeometricData.h>
#include <scene/data/GeometricDataVisitor.h>

#include <scene/data/geometric/GeometricDataFactory.h>
#include <scene/data/geometric/GeometricPoint.h>
#include <scene/data/geometric/Polygon2D.h>
#include <scene/data/geometric/Polygon3D.h>

#include <scene/model/ModelFactory.h>
#include <scene/model/PolygonRenderModel.h>

MeshConverterControl::MeshConverterControl(
        MeshConverterModule* module,
        ApplicationControl* ac)
    : mAc(ac)
    , mModule(module)
{
}

MeshConverterControl::~MeshConverterControl()
{

}

void MeshConverterControl::init()
{

}

void MeshConverterControl::loadGeometry2D(Polygon2D* /*poly2*/)
{

}

void MeshConverterControl::convert(
        const MeshCriteria& meshCriteria,
        bool renderOnlyOuterFaces)
{
    // get selected leaf nodes
    std::vector<std::shared_ptr<SceneLeafData>> sceneLeafData =
            mAc->getUIControl()->getSelectionControl()->retrieveSelectedSceneLeafData();

    mSavedPolygons.clear();

    class ExtractPolygonVisitor : public GeometricDataVisitor
    {
    public:
        ExtractPolygonVisitor()
        {
            polygon = nullptr;
        }

        virtual void visit(Polygon2D& polygon2D)
        {
            polygon = std::static_pointer_cast<Polygon>(
                        polygon2D.shared_from_this());
        }

        virtual void visit(Polygon3D& polygon3D)
        {
            polygon = std::static_pointer_cast<Polygon>(
                        polygon3D.shared_from_this());
        }

        virtual void visit(GeometricPoint& /*point*/) {}

        std::shared_ptr<Polygon> polygon;
    } extractPolygonVisitor;

    for (const std::shared_ptr<SceneLeafData>& leafData : sceneLeafData)
    {
        // save the current polygon
        leafData->getGeometricData()->accept(extractPolygonVisitor);

        Eigen::Vector3d position;

        std::shared_ptr<Polygon> poly = extractPolygonVisitor.polygon;
        if (poly)
        {
            // create new node
            SGLeafNode* newNode = mAc->getSGControl()->createLeafNode(
                        leafData->getNode()->getName() + " (converted)",
                        leafData->getNode()->getParent(),
                        poly);

            // create the new polygon
            mAc->getSGControl()->create3DGeometryFrom2D(
                        newNode,
                        meshCriteria,
                        renderOnlyOuterFaces);

            newNode->getData()->getGeometricData()->transform(poly->getTransform());
        }

        //TODO: save the newly created SGLeafNode* and the old Polygon
//        if (extractPolygonVisitor.polygon &&
//            extractPolygonVisitor.polygon != newNode->getData()->getGeometricData())
//        {
//            mSavedPolygons.push_back(
//                        std::make_tuple(
//                            newNode->getData(),
//                            extractPolygonVisitor.polygon));
//        }
    }
}

void MeshConverterControl::revert()
{
    // TODO: not implemented yet
    // save the old geometry
    // apply that geometry here with a corresponding method from sgControl
    // what is happening if a geometry is changed? corrensponding simulation object must be removed?
    // pop up a warning before this is happening?
    // Remove simulation object when its polygon changes?
    // How is there no segfault? simulation object keeps polygon alive i guess...

    for (std::tuple<
         std::shared_ptr<SceneLeafData>,
         std::shared_ptr<Polygon>>& p : mSavedPolygons)
    {
        std::shared_ptr<SceneLeafData> sceneData = std::get<0>(p);
        std::shared_ptr<Polygon> poly = std::get<1>(p);

        // when setting goemetric data, the corresponing render model must be
        // set as well.
        sceneData->setGeometricData(poly);

        std::shared_ptr<PolygonRenderModel> renderModel =
                ModelFactory::createPolygonRenderModelImproved(
                    mAc->getRenderModelManager(), poly, true);
        sceneData->setRenderModel(renderModel);
    }
    //    mAc->getSGControl()->
}
