#ifndef RENDERPOLYGONS_H
#define RENDERPOLYGONS_H

#include "RenderObject.h"

#include <data_structures/DataStructures.h>
#include <multi_threading/Monitor.h>
#include <proxy/ProxyDefs.h>
#include <scene/data/geometric/BSWSVectors.h>

class Domain;
class RenderPolygonsData;
class RenderPolygonsConstantData;
class RenderPolygonsConstantDataBS;
class RenderPolygonsConstantDataWS;
class RenderPolygonsProxy;

// Represents and renders multiple Polygons that share
// either their topology (Type == WORLD_SPACE) or their
// topology and their positions/ normals (Type == BODY_SPACE).
//
// There is one RenderPolygons object for all polygons that
// shared certain data. Deformables share their topology
// while rigid bodies share topology, vertex positions and normals.
// Render multiple polygons that share
// Type == BODY_SPACE: topology and positions/ normals but no transformation matrix
// Type == WORLD_SPACE: topology. They have no transform
class RenderPolygons : public RenderObject
{
public:

    // Construtor that creates RenderPolygons in body space coordinates.
    // \param domain - the Renderer domain
    // \param constantData - constant data in body space coordianates
    RenderPolygons(std::shared_ptr<RenderPolygonsConstantDataBS>& constantData);
    RenderPolygons(std::shared_ptr<RenderPolygonsConstantDataWS>& constantData);

    virtual ~RenderPolygons();

    void setDomain(Domain* domain);
    Domain* getDomain();

    BSWSVectors::Type getType() const;

    void addRenderPolygonsData(std::shared_ptr<RenderPolygonsData> data);
    void removeRenderPolygonsData(std::shared_ptr<RenderPolygonsData> data);

// slots:
    // Only call this method after calling setDomain(Domain*)
    void addRenderPolygonsDataSlot(std::shared_ptr<RenderPolygonsData> data);

    // Only call this method after calling setDomain(Domain*)
    void removeRenderPolygonsDataSlot(std::shared_ptr<RenderPolygonsData> data);

    std::vector<std::shared_ptr<RenderPolygonsData>>& getData();
    std::shared_ptr<RenderPolygonsConstantData>& getConstantData();

    // RenderObject interface
public:
    virtual void accept(RenderObjectVisitor& visitor);
    virtual void draw();
    virtual void drawImmediate();
    virtual void drawArray();
    virtual void drawVBO();
    virtual void update();
    virtual void createBuffers();
    virtual void refreshBuffers();
    virtual void cleanup();
    virtual void initialize();

private:

    Domain* mDomain;

    std::shared_ptr<RenderPolygonsProxy> mProxy;

    BSWSVectors::Type mType;

    // Shared data: can be shared among multiple RenderModels
    std::vector<std::shared_ptr<RenderPolygonsData>> mData;

    // Constant data: is unique for all RenderModels
    std::shared_ptr<RenderPolygonsConstantData> mConstantData;

    bool mVBOsupported;
};

PROXY_CLASS(RenderPolygonsProxy, RenderPolygons, mR,
            PROXY_FUNCTION(RenderPolygons, mR, addRenderPolygonsDataSlot,
                           PL(std::shared_ptr<RenderPolygonsData> data),
                           PL(data))
            PROXY_FUNCTION(RenderPolygons, mR, removeRenderPolygonsDataSlot,
                           PL(std::shared_ptr<RenderPolygonsData> data),
                           PL(data))
            )

#endif // RENDERPOLYGON_H
