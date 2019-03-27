#ifndef RENDERPOLYGONSCONSTANTDATA_H
#define RENDERPOLYGONSCONSTANTDATA_H


class RenderPolygonsConstantData
{
public:
    RenderPolygonsConstantData();
    virtual ~RenderPolygonsConstantData();

    virtual void initialize() = 0;
    virtual void cleanup() = 0;
    virtual void createBuffers() = 0;
    virtual void refreshBuffers() = 0;
    virtual bool isInitialized() const = 0;
};

#endif // RENDERPOLYGONSCONSTANTDATA_H
