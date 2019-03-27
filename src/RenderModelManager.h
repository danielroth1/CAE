#ifndef RENDERMODELMANAGER_H
#define RENDERMODELMANAGER_H

#include <map>
#include <memory>
#include <vector>


class ApplicationControl;
class PolygonData;
class Renderer;
class RenderModel;
class RenderPolygons;

// Allows to know if a Polygon2DData was already added to the renderer.
// If so, the corresponding RenderPolygons object can be retrieved.
class RenderModelManager
{
public:
    RenderModelManager(ApplicationControl* ac);

    void updateAllRenderModels();

    void addRenderModel(std::shared_ptr<RenderModel> renderModel);
    void removeRenderModel(std::shared_ptr<RenderModel> renderModel);

    // Adds a render model for the given object. The render model is stored
    // in this class and is removed by calling removeRenderModelByObject(object).
    // This allows to remove the renderModel from the object that is referenced by
    // the render model, thus, it is no longer necessary to manage the render model
    // itself.
    // Call this method before object needs to be removed.
    void addRenderModelByObject(std::shared_ptr<void> object,
                                std::shared_ptr<RenderModel> renderModel);

    // Removes the render model that was added for the given object
    // with addRenderModelByObject(object, renderModel).
    void removeRenderModelByObject(std::shared_ptr<void> object);

    std::shared_ptr<RenderPolygons> getRenderPolygons(
            std::shared_ptr<PolygonData> polygon2DData);

    void addPolygonData(std::shared_ptr<PolygonData> polygon2DData,
                          std::shared_ptr<RenderPolygons> renderPolygons);

    void removePolygonData(std::shared_ptr<PolygonData> polygon2DData);

private:

    void updateRenderModel(Renderer* renderer, const std::shared_ptr<RenderModel>& rm);

    ApplicationControl* mAc;

    std::vector<std::shared_ptr<RenderModel>> mRenderModels;
    std::map<std::shared_ptr<void>, std::shared_ptr<RenderModel>> mRenderModelsMap;

    std::map<   std::shared_ptr<PolygonData>,
                std::shared_ptr<RenderPolygons>> mPDataToRenderPolygonsMap;

};

#endif // RENDERMODELMANAGER_H
