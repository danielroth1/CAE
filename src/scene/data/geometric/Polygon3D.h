#ifndef POLYGON3D_H
#define POLYGON3D_H

#include "data_structures/DataStructures.h"
#include "Polygon.h"

#include <memory>

class Polygon3DData;
class Polygon3DTopology;
class PositionData;
class TopologyFeature;

class Polygon3D : public Polygon
{
public:

    // Constructor for world space position data.
    // Calculates the missing normals for the outer.
    // vertices.
    Polygon3D(
            const Vectors& positionsWS,
            const Polygon3DTopology& topology);

    // Constructor for world space position data.
    // The outer vertex normals are given.
    Polygon3D(
            const Vectors& positionsWS,
            const Vectors& vertexNormalsWS,
            const Polygon3DTopology& topology);

    // Constructor for body space position data
    // Calculates the missing normals for the outer.
    Polygon3D(
            Vectors* positionsBS,
            const Eigen::Affine3d& transform,
            const Polygon3DTopology& topology);

    // Constructor for body space position data
    // The outer vertex normals are given.
    Polygon3D(
            Vectors* positionsBS,
            const Eigen::Affine3d& transform,
            const Vectors& vertexNormalsBS,
            const Polygon3DTopology& topology);

    virtual ~Polygon3D() override;

    // Calculates a vector of positions whomes indices correspond to the
    // indices from Topology2D. This operation is expensive as a whole vector
    // is created each time it is called so don't use it in timing critical
    // code.
    Vectors calcualtePositions2DFrom3D() const;

    // Getters
    Polygon3DTopology& getTopology3D();
    const Polygon3DTopology& getTopology3D() const;

    // Returns a vector of IDs that point to the positions
    // that are part of the outer hull. The outer hull are
    // all vertices that are returned by getOuterFaces().
    // No ID is duplicated. The ids are ordered ascending.
    std::vector<unsigned int>& getOuterPositionIds();

    Vectors& getOuterVertexNormals();
    Vectors& getOuterFaceNormals();

    std::shared_ptr<Polygon3DData> getData3D();


    // GeometricData interface
public:
    void updateBoundingBox() override;
    void accept(GeometricDataVisitor& visitor) override;

    // Polygon interface
public:
    virtual void update() override;

    // Checks if the given point is inside the outer topology. Only tests the
    // faces that are part of the given feature.
    // Retruns false, if there are no faces.
    virtual bool isInside(
            const TopologyFeature& feature,
            Eigen::Vector point) override;

    virtual bool isInside(
            const TopologyFeature& feature,
            Vector source,
            double distance,
            Vector target) override;

    virtual DimensionType getDimensionType() const override;

    virtual std::shared_ptr<PolygonData> getData() override;

    virtual PolygonTopology& getTopology() override;

    virtual void changeRepresentationToBS(const Eigen::Vector& center) override;

    virtual void changeRepresentationToWS() override;

    virtual void setTransform(const Eigen::Affine3d& transform) override;

protected:

    bool isInside(ID faceId,
                  const Vector& point,
                  PolygonTopology& topology,
                  BSWSVectors& faceNormals) override;

private:

    std::shared_ptr<Polygon3DData> mData;

    BSWSVectors mOuterVertexNormals;
    BSWSVectors mOuterFaceNormals;

};

#endif // POLYGON3D_H
