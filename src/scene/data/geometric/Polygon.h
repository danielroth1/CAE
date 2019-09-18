#ifndef POLYGON_H
#define POLYGON_H

#include "BSWSVectors.h"
#include "PositionData.h"

#include <scene/data/GeometricData.h>

class Polygon2DAccessor;
class PolygonData;
class PolygonTopology;
class TopologyFace;
class TopologyFeature;

class Polygon : public GeometricData
{
public:

    enum class DimensionType
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

    // This method updates the world space positions.
    // Call this method when in BODY_SPACE representation type
    // and the transformation matrix changed.
    virtual void update() override;

    // Fix the topology by removing all vertices that are not referenced by
    // other topological elements like edges, faces, and cells. This is important
    // if algorithms are applied that rely on the fact that each vertex
    // is part of at least one of each other topological element.
    virtual void fixTopology() = 0;

    // Removes the vertex at the given index. This includes position and
    // normal.
    virtual void removeVertex(ID index);

    // Removes the vertices at the given index. This includes positions and
    // normals.
    virtual void removeVertices(std::vector<ID>& indices);

    // Checks if the given point is inside the topology. Only tests the
    // faces that are part of the given feature.
    // Retruns false, if there are no faces.
    virtual bool isInside(const TopologyFeature& feature, Eigen::Vector point) = 0;

    // Same as isInside(TopologyFeature&, Eigen::Vector) but searches all
    // faces that are within source and the given distance.
    // \param distance - the minimum distance from the given feature for which
    //      the isInside check should be accurate
    virtual bool isInside(
            const TopologyFeature& feature,
            Vector source,
            double distance,
            Vector target) = 0;

    virtual DimensionType getDimensionType() const = 0;

    virtual std::shared_ptr<PolygonData> getData() = 0;

    virtual PolygonTopology& getTopology() = 0;

    // Creates an accessor that allows to access this polygons outer mesh.
    virtual std::shared_ptr<Polygon2DAccessor> createAccessor() = 0;

    // GeometricData interface
public:
    virtual Type getType() const override;

    // Position with index
    // Always returns the world space representation.
    virtual Vector& getPosition(size_t index) override;

    // Sets the world space position. If the polygon is in body space
    // these changes, will be overwritten after the next update() call.
    virtual void setPosition(size_t index, const Eigen::Vector& position);

    // Number of positions
    virtual size_t getSize() override;

    // Translates all vertices/ the center by the given vector.
    virtual void translate(const Eigen::Vector& position) override;

    virtual void transform(const Eigen::Affine3d& transform) override;

    // Delegated PositionData methods
public:

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
    virtual void changeRepresentationToWS() = 0;

    const ID* getRelevantFaces(const TopologyFeature& feature, size_t& count);

protected:

    // Destructor
    virtual ~Polygon() override;

    // Checks if the given point is inside the topology. Only tests the
    // faces that are part of the given feature.
    // Returns false, if there are no faces.
    bool isInside(
            const TopologyFeature& feature,
            Eigen::Vector point,
            PolygonTopology& topology,
            BSWSVectors& faceNormals);

    // Checks if target is inside the polygon. This method doesn't traverse
    // all faces of the polygon but only the faces that are within the are
    // that reaches from source to distance. Also only faces that are connected
    // to the given topology are looked at.
    bool isInside(
            const TopologyFeature& feature,
            Vector source,
            double distance,
            Vector target,
            PolygonTopology& topology,
            BSWSVectors& faceNormals);

    virtual bool isInside(ID faceId,
                  const Vector& point,
                  PolygonTopology& topology,
                  BSWSVectors& faceNormals);

    // Checks if the point is within the distance of the face which is the
    // case if the point is whithin the distance of at least one vertex of the
    // face.
    bool isWithinDistance(
            TopologyFace& face,
            const Eigen::Vector& point,
            double distance);

    // Protected Members
//    PositionData mPositionData;
    PositionData mPositionData;

    // There is one normal per outer face. The normal is orthogonal to the plane and
    // points to the outside of the polygon.
    Vectors mNormals;

};

#endif // POLYGON_H
