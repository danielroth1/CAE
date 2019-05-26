#include "MeshConverter.h"
#include "MeshConverterControl.h"

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
#include <scene/model/PolygonRenderModelImproved.h>

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
        double facetAngle,
        double facetSize,
        double facetDistance,
        double cellSize,
        double cellRadiusEdgeRatio,
        bool renderOnlyOuterFaces)
{
    // get selected leaf nodes
    // TODO: how to retrieve the SGLeafNode from SceneLeafData? currently not possible
    // either get the SGLeafNode directly from the selection or store pointer in
    // SceneLeafData
    std::vector<std::shared_ptr<SceneLeafData>> sceneLeafData =
            mAc->getUIControl()->getSelectionControl()->retrieveSelectedSceneLeafData();

    mSavedPolygons.clear();

    class ExtractPolygonVisitor : public GeometricDataVisitor
    {
    public:
        ExtractPolygonVisitor()
        {

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
        extractPolygonVisitor.polygon = nullptr;
        leafData->getGeometricData()->accept(extractPolygonVisitor);

        // create the new polygon
        SGLeafNode* newNode =
                mAc->getSGControl()->create3DGeometryFrom2D(
                    static_cast<SGLeafNode*>(leafData->getNode()),
                    facetAngle,
                    facetSize,
                    facetDistance,
                    cellSize,
                    cellRadiusEdgeRatio,
                    renderOnlyOuterFaces);

        //TODO: save the newly created SGLeafNode* and the old Polygon
        if (extractPolygonVisitor.polygon &&
            extractPolygonVisitor.polygon != newNode->getData()->getGeometricData())
        {
            mSavedPolygons.push_back(
                        std::make_tuple(
                            newNode->getData(),
                            extractPolygonVisitor.polygon));
        }
    }

//    class ConverterVisitor : public GeometricDataVisitor
//    {
//    public:
//        ConverterVisitor(
//                    MeshConverterControl& _control,
//                    SGLeafNode* _leafNode,
//                    double _cellSize,
//                    double _cellRadiusEdgeRatio,
//                    bool _renderOnlyOuterFaces)
//            : control(_control)
//            , leafNode(_leafNode)
//            , cellSize(_cellSize)
//            , cellRadiusEdgeRatio(_cellRadiusEdgeRatio)
//            , renderOnlyOuterFaces(_renderOnlyOuterFaces)
//        {
//            control.mAc->getSGControl()->create3DGeometryFrom2D(
//                        leafNode, cellSize, cellRadiusEdgeRatio, renderOnlyOuterFaces);
//        }

//        virtual void visit(Polygon2D& /*polygon2D*/)
//        {
//            // Convert this polygon2D and set the resulting Polygon3D
//            // to as geometric data in node.

////            GeometricDataFactory::createPolygon3DFromPolygon2D(polygon2D, cellSize, cellRadiusEdgeRatio);
//            control.mAc->getSGControl()->create3DGeometryFrom2D(
//                        leafNode, cellSize, cellRadiusEdgeRatio, renderOnlyOuterFaces);
//        }

//        virtual void visit(Polygon3D& /*polygon3D*/)
//        {
//            // Convert the outer 2D mesh?
//            std::cout << "GeometricData is already Polygon3D. No conversion possible.\n";
//        }

//        virtual void visit(GeometricPoint& /*point*/)
//        {
//            std::cout << "Can not create Polygon3D from GeometricPoint.\n";
//        }

//        MeshConverterControl& control;
//        SGLeafNode* leafNode;
//        double cellSize;
//        double cellRadiusEdgeRatio;
//        bool renderOnlyOuterFaces;
//    } converterVisitor(*this, , cellSize, cellRadiusEdgeRatio, renderOnlyOuterFaces);


//    MeshConverter::instance()->generateMesh(
//                so->getPositions(), so->getFaces(),
//                vertices_out, outer_faces_out, faces_out, cells_out);
//    mMeshConverter->
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

        std::shared_ptr<PolygonRenderModelImproved> renderModel =
                ModelFactory::createPolygonRenderModelImproved(
                    mAc->getRenderModelManager(), poly, true);
        sceneData->setRenderModel(renderModel);
    }
    //    mAc->getSGControl()->
}
