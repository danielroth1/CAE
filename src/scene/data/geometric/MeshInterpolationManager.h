#ifndef MESHINTERPOLATIONMANAGER_H
#define MESHINTERPOLATIONMANAGER_H

#include <memory>
#include <vector>

class ApplicationControl;
class GeometricDataListener;
class MeshInterpolator;
class MeshInterpolatorRenderModel;
class Polygon;
class Polygon3D;
class Renderer;

class MeshInterpolationManager
{
public:
    // If no renderer is provided, this class is still functional. Only the
    // visualization won't work;
    MeshInterpolationManager(Renderer* renderer);

    // Adds a FEM based interpolation.
    //\return false if there is already an interpolation for the given target.
    //      Does nothing in that case.
    bool addInterpolatorFEM(const std::shared_ptr<Polygon3D>& source,
                             const std::shared_ptr<Polygon>& target);

    // Adds a MeshMesh based interpolation.
    //\return false if there is already an interpolation for the given target.
    //      Does nothing in that case.
    bool addInterpolatorMeshMesh(const std::shared_ptr<Polygon>& source,
                                  const std::shared_ptr<Polygon>& target);

    // Removes the interpolation for the given target.
    //\return true if there was such an interpolation that could be removed.
    bool removeInterpolator(const std::shared_ptr<Polygon>& target);

    void clearInterpolators();

    void setInterpolatorVisible(const std::shared_ptr<Polygon>& target,
                                 bool visible);

    // Returns the render model of the interpolator that has the given target.
    // If there is no such interpolator, nullptr is returned.
    std::shared_ptr<MeshInterpolatorRenderModel> getRenderModel(
            const std::shared_ptr<Polygon>& target);

    // Returns the interpolator that has the given polygon as target. If there
    // is none, returns nullptr.
    std::shared_ptr<MeshInterpolator> getInterpolator(
            const std::shared_ptr<Polygon>& target);

private:

    struct MeshInterpolationData
    {
        std::shared_ptr<MeshInterpolator> mInterpolator;
        std::shared_ptr<MeshInterpolatorRenderModel> mRenderModel;
        std::shared_ptr<GeometricDataListener> mListener;
    };

    // Unloads a MeshInterpolationData. Call this method before deleting such
    // a data. Makes sure that all remaining references on its members (like
    // the GeometricDataListener on the corresponding target geometry) are
    // properly removed.
    void unload(const std::shared_ptr<MeshInterpolationData>& data);

    std::shared_ptr<MeshInterpolatorRenderModel> createRenderModel(
            const std::shared_ptr<MeshInterpolator>& interpolator);

    // Creates a geometric data listener and adds it as listener to the
    // source GeometricData of the interpolator. Updates the interpolator
    // and renderModel each time the source geometric data of the interpolator
    // changes.
    std::shared_ptr<GeometricDataListener> createGeometricDataListener(
            const std::shared_ptr<MeshInterpolator>& interpolator,
            const std::shared_ptr<MeshInterpolatorRenderModel>& renderModel);

    std::shared_ptr<MeshInterpolationData> getData(
            const std::shared_ptr<Polygon>& target);

    // Creates the MeshInterpolationData for the given interpolator, initializes
    // all necessary data, and adds it to mData.
    // Only call this method if it was made sure that the interpolation can
    // be actually added (if there is no other interpolation with the given target).
    void addInterpolator(const std::shared_ptr<MeshInterpolator>& interpolator);

    Renderer* mRenderer;

    std::vector<std::shared_ptr<MeshInterpolationData>> mData;
};

#endif // MESHINTERPOLATIONMANAGER_H
