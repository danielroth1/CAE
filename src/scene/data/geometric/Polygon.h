#ifndef POLYGON_H
#define POLYGON_H

#include "BSWSVectors.h"
#include "PositionData.h"

#include <scene/data/GeometricData.h>

class PolygonData;


class  Polygon : public GeometricData
{
public:
    enum class Type
    {
        TWO_D, THREE_D
    };

    Polygon();

    // Creates polygon from world space positions.
    Polygon(const Vectors& positionsWS);

    // Creates polygon in body space representation.
    Polygon(Vectors* positionsBS, const Affine3d& transform);

    // Creates polygon from body space positions
    // and affine transformation.
    void initWorldSpace(
                const Vectors& positionsWS);

    void initBodySpace(
                Vectors* positionsBS,
                const Eigen::Affine3d& transform);

    void setFromWorldSpace(Vectors positionsWS);
    void setFromBodySpace(Vectors positionsBS);

    virtual void update();

    virtual Type getType() = 0;

    virtual std::shared_ptr<PolygonData> getData() = 0;

    // GeometricData interface
public:
    // Position with index
    virtual Vector& getPosition(size_t index) override;

    // Number of positions
    virtual size_t getSize() override;

    // Translates all vertices/ the center by the given vector.
    virtual void translate(const Eigen::Vector& position) override;

    // Delegated PositionData methods
public:

    // This method updates the world space positions.
    // Call this method when in BODY_SPACE representation type
    // and the transformation matrix changed.
    void updatePositions();

    BSWSVectors::Type getPositionType();

    // World space
    // Return the world space positoins vector.
    Vectors& getPositions();

    // Body space
    Eigen::Affine3d& getTransform();
    Vectors& getPositionsBS();
    Eigen::Vector& getPositionBS(ID index);

    // Offset that points for each vertex to its initial body space
    // By calling changeRepresentationToBS(-mCenter) the original
    // vertex positions of each vertex v_i can be calculate with
    // mTransform.inverse() * v_i.
    Eigen::Vector getCenter() const;

    virtual void setTransform(const Eigen::Affine3d& transform);

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
//    virtual void changeRepresentationToBS(
//            Vectors* vectorsBS,
//            const Eigen::Affine3d& transform);
    virtual void changeRepresentationToBS(const Eigen::Vector& center) = 0;

    // Changes the representation type to world space. In
    // this type, only the world space positions may be
    // changed. Transformation matrix and body space positions
    // can be updated by calling update().
    //
    // -> From body space to world space: applies the
    // transformation matrix on each vertex
    virtual void changeRepresentationToWS();

protected:

    // Calcualtes the edges for the given faces.
    // Avoids duplicated edges. For each edges with
    // its vertex indices (i0, i1) it is always
    // i0 < i1. This condition is part of the algorithm
    // and is true for the resulting edges as well.
    Edges calculateEdges(const Faces& faces);

    // Destructor
    virtual ~Polygon() override;

    // Protected Members
//    PositionData mPositionData;
    PositionData mPositionData;

    // There is one normal per outer face. The normal is orthogonal to the plane and
    // points to the outside of the polygon.
    Vectors mNormals;

};

#endif // POLYGON_H
