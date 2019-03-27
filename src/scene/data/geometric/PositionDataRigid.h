#ifndef GEOMETRICDATARIGID_H
#define GEOMETRICDATARIGID_H

#include <scene/data/GeometricData.h>


// -> A rigid has vertex positions in body space that are not changed.
// The only changes it should undergo is in its transformation
// matrix.
// -> Worldspace vertices needs to be updated when the transformation
// matrix changes. This is done with updateWorldSpacePositions().
// -> getPosition(int index) returns the vertex position at the given
// index in world space positions. To get the vertex position in
// body space, use getPositionBS(int index).
class GeometricDataRigid : public GeometricData
{
public:
    // Body and world space positions are equal at the start.
    // The addine transform is initialized as identity matrix.
    GeometricDataRigid(const Vectors& positionsBS);

    // Geometric Data Method
    // Returns the world space position at the given index.
    virtual Vector& getPosition(size_t index) override;

    // Geometric Data Method
    virtual size_t getSize() override;

    virtual Vector& getPositionBS(size_t index);

    // Returns the affine transformation of this geometric data.
    // The simulation can adapt this transform (used for the
    // simulation of rigid bodies). After adapting the transform
    // geometricDataChanged() should be called to inform
    // the renderer about the change.
    Eigen::Affine3d& getTransform();

    // Updates the world space positions according to the
    // current affine transformation, i.e.
    // v_ws = mTransform * v_bs
    virtual void updateWorldSpacePositions();

    // Sets the center of the vertices.
    // The center can be used to describe the center of
    // mass or the average of vertices. This method is
    // usefull for cases where the center of mass changes
    // or when the center vertex of imported data doesn't
    // make sense.
    //
    // Moves all body space positions by the difference
    // between new and old center. Applies a transformation
    // in the opposite direction. This means their
    // world space coordinates do not change, i.e.
    // v_bs = v_bs - center
    // transform.translate(-center)
    //
    // The center is not required to determine the world
    // space coordinates of positions.
    //
    // The original
    // center is at (0, 0, 0). To restore the data as it
    // was imported, cell this method with Eigen::Vector::Zeros().
    void setCenter(Eigen::Vector center);

private:
    Vectors mPositionsBS; // body space positions

    Vectors mPositionsWS; // world space positions

    Eigen::Affine3d mTransform;
};

#endif // GEOMETRICDATARIGID_H
