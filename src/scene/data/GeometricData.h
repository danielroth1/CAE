#ifndef GEOMETRICDATA_H
#define GEOMETRICDATA_H

#include <data_structures/BoundingBox.h>


class GeometricDataListener;
class GeometricDataVisitor;

// This is a container for any type of geometric data.
// It is stored in the scene graph.
// Data in this class is permanent and serializable.
// It is saved when the scene graph is written.
//
// Call geometricDataChanged() whenever changes were made
// to the data that other modules (like the renderer)
// should know about. This approach was chosen
// over a setter (e.g. setTransform(Eigen::Affine3d)) to
// 1. avoid unneccessary copying of data and
// 2. giving more control to the simulation about when
//    the renderer can access the data without knowing
//    anything about the renderer.
//
// Use getTransform() to adapt the affine transformation.
// (Affine3f is represented by a 4x4 matrix storing
// rotational and translatoric part. It can be adapted
// by using a concatenation of Eigen affine transformations
// or by direćtly using the methods in Affine3f (translate,
// rotate). Scaling is not recomended to not confuse the
// simulation.
//
// 2 Modes to represent positoins:
// Affine transformation mode:
// update affine transformation
// positionsBS - vectors that point from center to
//      vertex position
// mTransfom - affine transformation matrix. Transforms
//      vectors in positionsBS to their corresponding
//      world space corrdinates.
// 1.) body space positions + center + transformation
// 2.) world space positions
class GeometricData : public std::enable_shared_from_this<GeometricData>
{
public:
    GeometricData();
    virtual ~GeometricData();

    // Updates the bounding box.
    // The bounding box surrounds this whole object.
    virtual void updateBoundingBox() = 0;

    // Visitor method
    virtual void accept(GeometricDataVisitor& visitor) = 0;

    // Position with index
    virtual Vector& getPosition(size_t index) = 0;

    // Number of positions
    virtual size_t getSize() = 0;

    virtual void translate(const Eigen::Vector& position) = 0;

    // This method must be called when this objects geometric data changed.
    void geometricDataChanged();

    // Listener support
    void addGeometricDataListener(
            std::shared_ptr<GeometricDataListener> listener);

    bool removeGeometricDataListener(GeometricDataListener* listener);

    BoundingBox& getBoundingBox();

protected:

    BoundingBox mBoundingBox;

private:
    std::vector<std::shared_ptr<GeometricDataListener>> mListeners;
};

#endif // GEOMETRICDATA_H
