#ifndef BSWSVECTORS_H
#define BSWSVECTORS_H

#include <data_structures/DataStructures.h>
#include <data_structures/VectorOperations.h>
#include <iterator>

// Container for vertices either in body or in world
// space coordinates.
// Has the two storage modes: BSWSVectorsType
//  - BSWSVectorsType::BODY_SPACE - stores body space
//      vertex data and transformation matrix.
//      To access the correct world space positions with
//      getPositions() it is necessary to call update()
//      after changes in thetransofmraiton matrix.
//  - BSWSVectorsType::WORLD_SPACE - stores world space
//      vertex data. A update is usually not required.
//      Other components depend on the world space data,
//      Therefore, holding the body space data consistent
//      is not necessary.
//
// It is possible to change the representation type with
// changeRepresentationTypeToWS() and
// changeRepresentationTypeToBS()
//
// The indeded way of retrieving a position is to use
// getPosition(int index) or getPositions()
// If this object is in WORLD_SPACE, this is always possible.
// If its in BODY_SPACE, update() must be called before.
// If only a single position must be retrieved,
// getTransform() * getPositionBS(int index) should be used.
//
class BSWSVectors
{
public:

    enum Type
    {
        WORLD_SPACE, BODY_SPACE
    };

    // Noninitialized
    // Call either initializeFromWorldSpace or initializeFromBodySpace
    // before calling any other method.
    BSWSVectors();

    // World space representation of vector of the given size.
    // Content of vector is arbitrary.
    BSWSVectors(size_t size);

    // Creates positions data from world space positions.
    BSWSVectors(const Vectors& vectorsWS);

    // Creates positions data from body space positions
    // and affine transformation.
    BSWSVectors(
            Vectors* vectorsBS,
            const Eigen::Affine3d& transform);

    // Copy constructor
    BSWSVectors(const BSWSVectors& posData);

    // This method updates the world space positions.
    // Call this method when in BODY_SPACE representation type
    // and the transformation matrix changed.
    void update();

    Type getType()
    {
        return mType;
    }

    size_t getSize()
    {
        return mVectorsWS.size();
    }

    template<class BidirIter>
    void removeVectors(BidirIter begin, BidirIter end)
    {
        VectorOperations::removeVectors(mVectorsWS, begin, end);
    }

    void removeVector(ID index);

    // World space
    Vectors& getVectors()
    {
        return mVectorsWS;
    }
    const Vectors& getVectors() const
    {
        return mVectorsWS;
    }
    void setVectors(const Vectors& vectors)
    {
        mVectorsWS = vectors;
    }

    Eigen::Vector& getVector(ID index)
    {
        return mVectorsWS[index];
    }
    const Eigen::Vector& getVector(ID index) const
    {
        return mVectorsWS[index];
    }
    void setVector(ID index, const Eigen::Vector& v)
    {
        mVectorsWS[index] = v;
    }

    // Body space
    Eigen::Affine3d& getTransform()
    {
        return mTransform;
    }

    Vectors& getVectorsBS()
    {
        return *mVectorsBS;
    }
    const Vectors& getVectorsBS() const
    {
        return *mVectorsBS;
    }
    void setVectorsBS(const Vectors& vectors)
    {
        *mVectorsBS = vectors;
    }

    Eigen::Vector& getVectorBS(ID index)
    {
        return (*mVectorsBS)[index];
    }
    const Eigen::Vector& getVectorBS(ID index) const
    {
        return (*mVectorsBS)[index];
    }
    void setVectorBS(ID index, const Eigen::Vector& v)
    {
        (*mVectorsBS)[index] = v;
    }

    void setTransform(const Eigen::Affine3d& transform)
    {
        mTransform = transform;
    }

    // Changes the representation type to body space. In
    // this type, only the transformation matrix may be
    // changed. World space positions can be updated by
    // calling update().
    //
    // -> From world to body space: Requires the new center.
    //
    // \param center - One can calculate either the centroid
    //      with calculateCenterVertex() or the center of mass
    //      with calculateCenterOfMass(). By passing
    //      Eigen::Vector::Zero() vertices are not adapted,
    //      resulting in body space == world space coordinates.
    //      (for as long as there is no change in the transformation
    //      matrix and update() wasn't called.)
    void changeRepresentationToBS(Vectors* vectorsBS,
                                  const Eigen::Affine3d& transform);

    // Changes the representation type to world space. In
    // this type, only the world space positions may be
    // changed. Transformation matrix and body space positions
    // can be updated by calling update().
    //
    // -> From body space to world space: applies the
    // transformation matrix on each vertex
    void changeRepresentationToWS();

    // Translates all vertices/ the center by the given vector.
    void translate(const Eigen::Vector& t);

    void transform(const Eigen::Affine3d& transform);

    void updateWorldSpace();

    void moveCenter(Eigen::Vector center);

    void initializeFromWorldSpace(Vectors vectorsWS);

    void initializeFromBodySpace(
            Vectors* vectorsBS,
            const Eigen::Affine3d& transform);
private:

    Vectors* mVectorsBS; // body space positions

    Vectors mVectorsWS; // world space positions
    Eigen::Affine3d mTransform;

    Type mType;

    bool mInitialized;
};

#endif // BSWSVECTORS_H
