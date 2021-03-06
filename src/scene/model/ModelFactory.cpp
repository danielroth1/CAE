#include "ModelFactory.h"
#include "PolygonRenderModel.h"

#include <data_structures/DataStructures.h>
#include <scene/data/GeometricDataVisitor.h>
#include <scene/data/geometric/Polygon2D.h>
#include <scene/data/geometric/Polygon3D.h>

#include <RenderModelManager.h>
#include <iostream>
#include <memory>
#include <set>
#include <vector>


ModelFactory::ModelFactory()
{

}

std::shared_ptr<PolygonRenderModel> ModelFactory::createPolygonRenderModelImproved(
        RenderModelManager* rmm,
        std::shared_ptr<AbstractPolygon> poly,
        bool renderOnlyOuterFaces)
{
    class PolygonVisior : public GeometricDataVisitor
    {
    public:
        PolygonVisior(RenderModelManager* _rmm, bool _renderOnlyOuterFaces)
            : rmm(_rmm)
            , renderOnlyOuterFaces(_renderOnlyOuterFaces)
        {

        }

        virtual void visit(Polygon2D& polygon2D)
        {
            returnValue = ModelFactory::createPolygonRenderModelImproved(
                        rmm,
                        std::static_pointer_cast<Polygon2D>(polygon2D.shared_from_this()));
        }

        virtual void visit(Polygon3D& polygon3D)
        {
            returnValue = ModelFactory::createPolygonRenderModelImproved(
                        rmm,
                        std::static_pointer_cast<Polygon3D>(polygon3D.shared_from_this()),
                        renderOnlyOuterFaces);
        }

        virtual void visit(GeometricPoint& /*point*/)
        {
            std::cout << "Can not create a PolygonRenderModel from a GeometricPoint.\n";
        }

        RenderModelManager* rmm;
        bool renderOnlyOuterFaces;
        std::shared_ptr<PolygonRenderModel> returnValue;
    } v(rmm ,renderOnlyOuterFaces);

    poly->accept(v);
    return v.returnValue;
}

std::shared_ptr<PolygonRenderModel> ModelFactory::createPolygonRenderModelImproved(
        RenderModelManager* rmm,
        std::shared_ptr<Polygon2D> poly2)
{
    return std::make_shared<PolygonRenderModel>(rmm, poly2);
}

std::shared_ptr<PolygonRenderModel> ModelFactory::createPolygonRenderModelImproved(
        RenderModelManager* rmm,
        std::shared_ptr<Polygon3D> poly3,
        bool renderOnlyOuterFaces)
{
    return std::make_shared<PolygonRenderModel>(rmm, poly3, renderOnlyOuterFaces);
}
