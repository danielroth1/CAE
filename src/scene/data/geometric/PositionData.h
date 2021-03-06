#ifndef POSITIONDATA_H
#define POSITIONDATA_H

#include "BSWSVectors.h"

#include <data_structures/DataStructures.h>

// Container for vertices either in body or in world
// space coordinates.
// Has the two storage modes: PositionDataType
//  - PositionDataType::BODY_SPACE - stores body space
//      vertex data and transformation matrix.
//      To access the correct world space positions with
//      getPositions() it is necessary to call update()
//      after changes in thetransofmraiton matrix.
//  - PositionDataType::WORLD_SPACE - stores world space
//      vertex data. A update is usually not required.
//      Other components depend on the world space data,
//      Therefore, holding the body space data consistent
//      is not necessary.
//
// It is possible to change the representation type with
// changeRepresentationTypeToWS() and
// changeRepresentationTypeToBS().
//
class PositionData
{
public:

    // Default constructor.
    // Before initializeFromWorldSpace() or
    // initializeFromBodySpace() before calling
    // any other method.
    PositionData();

    // Creates positions data from world space positions.
    PositionData(const Vectors& positionsWS);

    // Creates positions data from body space positions
    // and affine transformation.
    PositionData(
            Vectors* positionsBS,
            const Eigen::Affine3d& transform);

    // Copy constructor
    PositionData(const PositionData& posData);

    // This method updates the world space positions.
    // Call this method when in BODY_SPACE representation type
    // and the transformation matrix changed.
    void update();

    // Removes a position at the given index.
    virtual void removePosition(ID index);

    // Removes the positions at the given indices.
    virtual void removePositions(const std::vector<ID>& indices);

    BSWSVectors::Type getType()
    {
        return mPositions.getType();
    }

    size_t getSize()
    {
        return mPositions.getSize();
    }

    // World space
    Vectors& getPositions()
    {
        return mPositions.getVectors();
    }

    const Vectors& getPositions() const
    {
        return mPositions.getVectors();
    }

    Eigen::Vector& getPosition(ID index)
    {
        return mPositions.getVector(index);
    }

    // If in body space representation, returns the temporary world space
    // positions.
    // If in world space representation, returns the regular permanent positions.
    const Eigen::Vector& getPosition(ID index) const
    {
        return mPositions.getVector(index);
    }

    // Sets the position at the given index. Use getSize() to see if the
    // index is within range.
    // - If in BODY_SPACE, sets the body space position. Note the difference
    //      behavior to the getter.
    // - If in WORLD_SPACE, sets the world space position
    void setPosition(ID index, const Eigen::Vector& position)
    {
        mPositions.setVector(index, position);
    }

    // Sets all positions.
    // - If in BODY_SPACE, sets the body space positions. Note the difference
    //      behavior to the getter.
    // - If in WORLD_SPACE, sets the world space positions.
    void setPositions(const Vectors& positions)
    {
        mPositions.setVectors(positions);
    }

    // Body space
    Eigen::Affine3d& getTransform()
    {
        return mPositions.getTransform();
    }
    Vectors& getPositionsBS()
    {
        return mPositions.getVectorsBS();
    }
    Eigen::Vector& getPositionBS(ID index)
    {
        return mPositions.getVectorBS(index);
    }

    // Offset that points for each vertex to its initial body space
    // By calling changeRepresentationToBS(-mCenter) the original
    // vertex positions of each vertex v_i can be calculate with
    // mTransform.inverse() * v_i.
    Eigen::Vector getCenter() const
    {
        return mCenter;
    }

    void setTransform(const Eigen::Affine3d& transform)
    {
        mPositions.setTransform(transform);
    }

    // Calculates the center vertex by averaging over
    // all world space vertices. This method is indended
    // to be used to calculate the center to change the
    // transformation type from world to body space with
    // changeRepresentationToBS().
    // If there is a uniform mass distribution
    // among all vertices, then this center is the center
    // of mass. If there is not, the center should probably
    // be calculated.
    Eigen::Vector calculateCenterVertex();

    // Calculates the center of mass. Also see
    // calculateCenterVertex().
    //\param masses - containts mass of each vertex. Must be
    //      of the size of the number of vertices.
    Eigen::Vector calculateCenterOfMass(
            const std::vector<double>& masses);

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
    void changeRepresentationToBS(
            Vectors* vectorsBS,
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

    void initializeFromWorldSpace(Vectors positionsWS);
    void initializeFromBodySpace(
            Vectors* positoinsBS,
            const Eigen::Affine3d& transform);

    void moveCenterTo(const Eigen::Vector& center);

    void moveCenter(const Eigen::Vector& deltaCenter);
private:

    void updateWorldSpace();

    BSWSVectors mPositions;

    // Offset that points for each vertex to its initial body space
    // By calling changeRepresentationToBS(-mCenter) the original
    // vertex positions of each vertex v_i can be calculate with
    // mTransform.inverse() * v_i.
    Eigen::Vector mCenter;
};

#endif // POSITIONDATA_H
