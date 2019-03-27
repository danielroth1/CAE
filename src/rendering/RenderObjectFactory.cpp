#include "RenderObjectFactory.h"

#include <rendering/object/RenderPolygons.h>


RenderObjectFactory::RenderObjectFactory(Domain* domain)
    : mDomain(domain)
{

}

std::shared_ptr<RenderPolygons>
RenderObjectFactory::createRenderPolygons(
        std::shared_ptr<RenderPolygonsConstantDataBS>& constantData)
{
    std::shared_ptr<RenderPolygons> renderPolygons =
            std::make_shared<RenderPolygons>(constantData);
    renderPolygons->setDomain(mDomain);
    return renderPolygons;
}

std::shared_ptr<RenderPolygons>
RenderObjectFactory::createRenderPolygons(
        std::shared_ptr<RenderPolygonsConstantDataWS>& constantData)
{
    std::shared_ptr<RenderPolygons> renderPolygons =
            std::make_shared<RenderPolygons>(constantData);
    renderPolygons->setDomain(mDomain);
    return renderPolygons;
}

