#ifndef POLYGONRENDERMODELIMPROVED_H
#define POLYGONRENDERMODELIMPROVED_H

#include "RenderModel.h"
#include <data_structures/DataStructures.h>
#include <memory>
#include <multi_threading/Monitor.h>
#include <rendering/buffers/BufferedData.h>
#include <scene/data/geometric/BSWSVectors.h>
#include <scene/data/geometric/TopologyFace.h>

class Appearances;
class GeometricDataListener;
class Polygon;
class PolygonIndexMapping;
class RenderLines;
class RenderObjectFactory;
class RenderPoints;
class RenderPolygons;
class RenderPolygonsData;
class RenderModelManager;

// Supports texturing:
// - allows one vertex to have multiple texture coordinates/ appearances
//      by using a PolygonIndexMapping. Each vertex is duplicated by each
//      additional assigned material. Duplicated vertices positions are updated
//      in updatePositions() accordingly by using the PolygonIndexMapping.
//
class PolygonRenderModel : public RenderModel
{
public:

    // A PolygonRenderModel is a render model that can be used for both
    // Polygon2D and Polygon3D.
    // \param renderOnlyOuterFaces - if true, only the outer faces of a Polygon3D are renderd
    //          if the given Polygon is of type Polygon2D, this variable has no effect.
    // \param renderVertexNormals - render the vertex normals. This can be a costly
    //          operation if there are a lot of faces because for each face a line
    //          is rendered and rendering lines currently is rather slow.
    // \param renderFaceNormal - this is even slower than renderVertexNormals for
    //          the same reasons and additionally it requires the recalculation
    //          of all face normals in each update step.
    //          The last part can be avoided for rigid bodies but this isn't
    //          implemented yet.
    //
    // Note: currently the vertex and (if renderFaceNormal) face normals are
    // recalculated in each update() step. This is only necessary for the simulation
    // of Polygons in WORLD_SPACE (e.g. deformables) and not polygons in
    // BODY_SPACE representation (e.g. rigids).
    //
    PolygonRenderModel(
            RenderModelManager* renderModelManager,
            std::shared_ptr<Polygon> polygon,
            bool renderOnlyOuterFaces = true,
            bool renderVertexNormals = false,
            bool renderFaceNormals = false);

    virtual ~PolygonRenderModel() override;

    // Sets the texture coordinates and sets a signal to refreshe the buffer
    // of them.
    void setTextureCoordinates(const std::vector<Eigen::Vector2f>& textureCoordinates);

    // Sets the rendered appearance. Does not overwrite this class appearance.
    virtual void setRenderedAppearances(
            const std::shared_ptr<Appearances>& appearances) override;

    virtual std::shared_ptr<Appearances> getRenderedAppearances() override;

    void setPolygonIndexMapping(
            const std::shared_ptr<PolygonIndexMapping>& polygonIndexMapping);

    bool isTexturingEnabled() const;

    // Sets texturing enables if:
    //  - the texture was specified via setTexture.
    //  - the texture coordinates were specified via setTextureCoordinates.
    // If not, sets texturingEnabled false and prints a message.
    void setTexturingEnabled(bool textureEnabled);

    bool isRenderOnlyOuterFaces() const;

    // Sets the flag for rendering only outer faces and resets() to make the
    // changes take effect.
    void setRenderOnlyOuterFaces(bool renderOnlyOuterFaces);

    bool isRenderVertexNormals() const;
    void setRenderVertexNormals(bool renderVertexNormals);

    bool isRenderFaceNormals() const;
    void setRenderFaceNormals(bool renderFaceNormals);

    // RenderModel interface
public:

    // Resizes positions, normals, and faces with the faces from mPolygon
    virtual void reset() override;
    virtual void update() override;
    virtual void revalidate() override;
    virtual void accept(RenderModelVisitor& v) override;
    virtual void addToRenderer(Renderer* renderer) override;
    virtual void removeFromRenderer(Renderer* renderer) override;
    virtual void setVisible(bool visible) override;
    virtual void setWireframeEnabled(bool wireframeEnabled) override;

private:

    // Retrieves either faces from Polygon2D or
    // outer faces from Polygon3D. Retrieved faces
    // are stored in the parameter faces.
    std::vector<TopologyFace>* retrieveRelevantFaces();

    void initializeBufferedData();

    void revalidatePointLineRendering();

    // Update mPositions with either the world or body space positions
    // from the geometric data depending on the representation type
    // of the geometric data.
    // When using the body space representation, call this method only
    // once after the center of the body space positions is set. Only
    // then the body space positions are correct.
    // Call it again when the center changes.
    void updatePositions();

    void updateNormalLines();

    void updateTransform();

    RenderModelManager* mRenderModelManager;

    Renderer* mRenderer;

    std::shared_ptr<GeometricDataListener> mGeometricListener;

    std::shared_ptr<Polygon> mPolygon;

    // Rendering the Polygon
    std::shared_ptr<RenderPolygons> mRenderPolygons;
    std::shared_ptr<RenderPolygonsData> mRenderPolygonsData;
    std::shared_ptr<PolygonIndexMapping> mPolygonIndexMapping;

    BufferedData<Eigen::Vectorf, float, 3>* mPositionsBufferedData;
    BufferedData<Eigen::Vectorf, float, 3>* mNormalsBufferedData;
    BufferedData<Face, unsigned int, 3>* mFacesBufferedData;
    BufferedData<Eigen::Vector2f, float, 2>* mTexturesCoordinatesBufferedData;

    // Rendering vertex normals
    std::shared_ptr<RenderLines> mRenderLinesNormals;
    std::shared_ptr<RenderPoints> mRenderPoints;

    BSWSVectors::Type mCurrentType;

    bool mRequiresUpdate;

    // If this variable is true, only the outer faces of a Polygon3D are renderd.
    // If the given polygon is a Polygon2D, this variable has no effect.
    bool mRenderOnlyOuterFaces;

    bool mTexturingEnabled;

    bool mRenderVertexNormals;
    bool mRenderFaceNormals;

    // Detecting transformation changes
    Eigen::Affine3d mCurrentlyRenderedTransform;
    Eigen::Vector mCurrentlyRenderedCenter;
};

#endif // POLYGONRENDERMODELIMPROVED_H
