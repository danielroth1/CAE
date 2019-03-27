#ifndef POLYGON3D_H
#define POLYGON3D_H

#include "data_structures/DataStructures.h"
#include "Polygon.h"

#include <memory>

class Polygon3DData;
class PositionData;

class Polygon3D : public Polygon
{
public:

    // Constructor for world space position data.
    // Calculates the missing normals for the outer.
    // vertices.
    Polygon3D(
            const Vectors& positionsWS,
            const Faces& faces,
            const Faces& outerFaces,
            const Cells& cells);

    // Constructor for world space position data.
    // The outer vertex normals are given.
    Polygon3D(
            const Vectors& positionsWS,
            const Vectors& vertexNormalsWS,
            const Faces& faces,
            const Faces& outerFaces,
            const Cells& cells);

    // Constructor for body space position data
    // Calculates the missing normals for the outer.
    Polygon3D(
            Vectors* positionsBS,
            const Eigen::Affine3d& transform,
            const Faces& faces,
            const Faces& outerFaces,
            const Cells& cells);

    // Constructor for body space position data
    // The outer vertex normals are given.
    Polygon3D(
            Vectors* positionsBS,
            const Eigen::Affine3d& transform,
            const Vectors& vertexNormalsBS,
            const Faces& faces,
            const Faces& outerFaces,
            const Cells& cells);

    virtual ~Polygon3D() override;

    // Getters
    Edges& getEdges();
    Edges& getOuterEdges();
    Faces& getFaces();
    Faces& getOuterFaces();
    Cells& getCells();

    // Returns a vector of IDs that point to the positions
    // that are part of the outer hull. The outer hull are
    // all vertices that are returned by getOuterFaces().
    // No ID is duplicated. The ids are ordered ascending.
    std::vector<unsigned int>& getOuterPositionIds();

    std::shared_ptr<Polygon3DData> getData3D();

    // GeometricData interface
public:
    void updateBoundingBox() override;
    void accept(GeometricDataVisitor& visitor) override;

    // Polygon interface
public:
    virtual Type getType() override;

    virtual std::shared_ptr<PolygonData> getData() override;

    virtual void changeRepresentationToBS(const Eigen::Vector& center) override;

    virtual void changeRepresentationToWS() override;

    virtual void setTransform(const Eigen::Affine3d& transform) override;

private:

    // Calculates the IDs of all vertices that are part of the
    // outer hull, which are all vertices that are obtained by
    // mOuterFaces. No index is duplicated.
    std::vector<unsigned int> calculateOuterPositionIDs(const Faces& faces);

    std::shared_ptr<Polygon3DData> mData;

    BSWSVectors mOuterVertexNormals;
    BSWSVectors mOuterFaceNormals;

};

#endif // POLYGON3D_H
