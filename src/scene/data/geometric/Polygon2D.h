#ifndef POLYGON2D_H
#define POLYGON2D_H

#include "Polygon.h"
#include "data_structures/DataStructures.h"

class Polygon2DData;
class Polygon2DTopology;

// Shared Data Policy:
// BODY_SPACE: either creates shared data or reuses it when the copy
//      constructor is called.
// WORLD_SPACE: shared data is lightweight. Is also created and reused
//      but there is basically no overhead.
class Polygon2D : public Polygon
{
public:
    // Constructor for world space position data
    // Calculates normals.
    Polygon2D(const Vectors& positionsWS,
              const Faces& faces);

    // Constructor for world space position data
    // Pass normals.
    Polygon2D(const Vectors& positionsWS,
              const Vectors& vertexNormalsWS,
              const Faces& faces);

    // Constructor for body space position data
    // Calculates normals.
    // Creates new Polygon2DData.
    Polygon2D(const Vectors& positionsBS,
              const Eigen::Affine3d& transform,
              const Faces& faces);

    // Constructor for body space position data
    // Pass normals.
    // Creates new Polygon2DData.
    Polygon2D(const Vectors& positionsBS,
              const Eigen::Affine3d& transform,
              const Vectors& vertexNormalsBS,
              const Faces& faces);

    Polygon2DTopology& getTopology();
    const Polygon2DTopology& getTopology() const;

    std::shared_ptr<Polygon2DData> getData2D();

    virtual void update() override;

    virtual std::shared_ptr<PolygonData> getData() override;

    // GeometricData interface
public:
    void accept(GeometricDataVisitor& visitor) override;
    void updateBoundingBox() override;

    // Polygon interface
public:
    virtual Type getType() override;

//    // Is this mehtod even needed?
//    // It allows to change the representation type but requires to
//    // know the location of positionBS in Polygon2DData but this is
//    // usually not known from outside.
//    virtual void changeRepresentationToBS(
//            Vectors* vectorsBS,
//            const Eigen::Affine3d& transform) override;

    // Copies the shared Polygon2DDataBS and creates a
    // Polygon2DDataWS.
    // Only has an effect if the object is in world space.
    //
    // Initializes the transformation matrices as identity matrices
    // and doesn't change the location of the vertex position.
    // This means that the current coordinate origin will be the
    // origin of the body space coordinate system of the polygon.
    virtual void changeRepresentationToBS(const Eigen::Vector& center) override;

    virtual void changeRepresentationToWS() override;

    virtual void setTransform(const Eigen::Affine3d& transform) override;

protected:
    Vectors& getVertexNormals();
    Vectors& getFaceNormals();

private:

    std::shared_ptr<Polygon2DData> mData;

    // These members are used to either store
    // the vectros in world space coordinates
    // or to store pointer to the vectors in body
    // space coordinates and offer a method "update()"
    // that converts those to world space by multiplying
    // each vector with the their transformation matrix.

    BSWSVectors mVertexNormals;
    BSWSVectors mFaceNormals; // currently unused
};

#endif // POLYGON2D_H
