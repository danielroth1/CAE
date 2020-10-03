#ifndef MODELFACTORY_H
#define MODELFACTORY_H

#include <memory>

class AbstractPolygon;
class Polygon2D;
class Polygon3D;
class PolygonRenderModel;
class RenderModelManager;

class ModelFactory
{
public:
    ModelFactory();

    // Creates a PolygonRenderModel, if its a Polygon2D everything is fine,
    // if its a Polygon3D \param renderOnlyOuterFaces decides if outer faces
    // are only rendered or all of them.
    static std::shared_ptr<PolygonRenderModel> createPolygonRenderModelImproved(
            RenderModelManager* rmm,
            std::shared_ptr<AbstractPolygon> poly,
            bool renderOnlyOuterFaces);

    // Creates a PolygonRenderModel for a Polygon2D:
    static std::shared_ptr<PolygonRenderModel> createPolygonRenderModelImproved(
            RenderModelManager* rmm,
            std::shared_ptr<Polygon2D> poly2);

    // Creates a PolygonRenderModel for a Polygon3D.
    // \param renderOnlyOuterFaces - if true onyl the outer faces of the Polygon3D
    //      are rendered.
    static std::shared_ptr<PolygonRenderModel> createPolygonRenderModelImproved(
            RenderModelManager* rmm,
            std::shared_ptr<Polygon3D> poly3,
            bool renderOnlyOuterFaces);
};

#endif // MODELFACTORY_H
