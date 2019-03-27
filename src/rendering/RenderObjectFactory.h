#ifndef RENDEROBJECTFACTORY_H
#define RENDEROBJECTFACTORY_H


#include <memory>

class Domain;
class RenderPolygons;
class RenderPolygonsConstantDataBS;
class RenderPolygonsConstantDataWS;

class RenderObjectFactory
{
public:
    RenderObjectFactory(Domain* domain);

    std::shared_ptr<RenderPolygons> createRenderPolygons(
            std::shared_ptr<RenderPolygonsConstantDataBS>& constantData);

    std::shared_ptr<RenderPolygons> createRenderPolygons(
            std::shared_ptr<RenderPolygonsConstantDataWS>& constantData);

private:
    Domain* mDomain;
};

#endif // RENDEROBJECTFACTORY_H
