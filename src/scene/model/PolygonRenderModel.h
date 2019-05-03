#ifndef POLYGONRENDERMODEL_H
#define POLYGONRENDERMODEL_H

#include "RenderModel.h"

#include <data_structures/DataStructures.h>
#include <memory>
#include <multi_threading/Monitor.h>
#include <scene/data/geometric/TopologyFace.h>
#include <vector>

class GeometricDataListener;
class GeometricDataVisitor;
class Polygon;
class RenderPoints;
class RenderLines;
class RenderPolygon2D;

class PolygonRenderModel : public RenderModel
{
public:
    // A PolygonRenderModel is a render model that can be used for both
    // Polygon2D and Polygon3D.
    // \param renderOnlyOuterFaces - if true, only the outer faces of a Polygon3D are renderd
    //          if the given Polygon is of type Polygon2D, this variable has no effect.
    PolygonRenderModel(std::shared_ptr<Polygon> polygon, bool renderOnlyOuterFaces = true);

    virtual ~PolygonRenderModel() override;

    // RenderModel interface
public:

    // Resizes mRenderObjectPositions, mRenderObjectNormal
    // Initializes mRenderObjectFaces with the faces from mPolygon
    virtual void reset() override;
    virtual void update() override;
    virtual void revalidate() override;
    virtual void accept(RenderModelVisitor& v) override;
    virtual void addToRenderer(Renderer* renderer) override;
    virtual void removeFromRenderer(Renderer* renderer) override;
    virtual void setVisible(bool visible) override;

protected:

    // Retrieves either faces from Polygon2D or
    // outer faces from Polygon3D. Retrieved faces
    // are stored in the parameter faces.
    std::vector<TopologyFace>* retrieveRelevantFaces();

    // Update mPositions with either the world or body space positions
    // from the geometric data depending on the representation type
    // of the geometric data.
    // When using the body space representation, call this method only
    // once after the center of the body space positions is set. Only
    // then the body space positions are correct.
    // Call it again when the center changes.
    void updatePositions();

    void updateNormalLines();

    std::shared_ptr<Polygon> mPolygon;

    std::shared_ptr<GeometricDataListener> mGeometricListener;
    std::shared_ptr<RenderPolygon2D> mRenderPoly2;
    std::shared_ptr<RenderLines> mRenderLinesNormals;
    std::shared_ptr<RenderPoints> mRenderPoints;

    // These is the data of the RenderPolygon2D that needs
    // to be adapted in the update method.
//    Vectorfs* mRenderObjectPositions;
//    Vectorfs* mRenderObjectNormals;
//    Faces* mRenderObjectFaces;
    Monitor<Vectorfs>* mRenderObjectPositions;
    Monitor<Vectorfs>* mRenderObjectNormals;
    Monitor<Faces>* mRenderObjectFaces;

    Eigen::Affine3d mCurrentlyRenderedTransform;
    Eigen::Vector mCurrentlyRenderedCenter;

    bool mRequiresUpdate;

    // If this variable is true, only the outer faces of a Polygon3D are renderd.
    // If the given polygon is a Polygon2D, this variable has no effect.
    bool mRenderOnlyOuterFaces;

};

#endif // POLYGONRENDERMODEL_H
