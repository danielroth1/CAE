#ifndef POLYGON3D_H
#define POLYGON3D_H

#include "data_structures/DataStructures.h"
#include "Polygon.h"
#include "Polygon2DAccessor.h"

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
            const std::shared_ptr<Polygon3DTopology>& topology);

    // Constructor for world space position data.
    // The outer vertex normals are given.
    Polygon3D(
            const Vectors& positionsWS,
            const Vectors& vertexNormalsWS,
            const std::shared_ptr<Polygon3DTopology>& topology);

    // Constructor for body space position data
    // Calculates the missing normals for the outer.
    Polygon3D(
            Vectors* positionsBS,
            const Eigen::Affine3d& transform,
            const std::shared_ptr<Polygon3DTopology>& topology);

    // Constructor for body space position data
    // The outer vertex normals are given.
    Polygon3D(
            Vectors* positionsBS,
            const Eigen::Affine3d& transform,
            const Vectors& vertexNormalsBS,
            const std::shared_ptr<Polygon3DTopology>& topology);

    virtual ~Polygon3D() override;

    // Calculates a vector of positions whomes indices correspond to the
    // indices from Topology2D. This operation is expensive as a whole vector
    // is created each time it is called so don't use it in timing critical
    // code.
    Vectors calcualtePositions2DFrom3D() const;

    // Creates .ele, .node, and .face files according to the tetgen file format.
    void outputToFile(const std::string& filename);

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
    virtual void update(bool updateFaceNormals = true,
                        bool updateVertexNormals = true,
                        bool notifyListeners = true) override;

    // Fix the topology by removing all vertices that are not referenced by
    // other topological elements like edges, faces, and cells. This is important
    // if algorithms are applied that rely on the fact that each vertex
    // is part of at least one of each other topological element.
    virtual void fixTopology() override;

    // Removes the vertex at the given index. This includes position and
    // normal.
    virtual void removeVertex(ID index) override;

    // Removes the vertices at the given index. This includes positions and
    // normals.
    virtual void removeVertices(std::vector<ID>& indices) override;

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

    // Returns an accessor that allows to access this polygons outer mesh.
    virtual const std::shared_ptr<Polygon2DAccessor>& getAccessor2D() override;

    // Creates and returns a Polygon2DAccessor for the outer polygon mesh.
    // This accessor allows to access the outer polygon mesh just like a
    // Polygon2D with its respective Polygon2DAccessor.
    virtual std::shared_ptr<Polygon2DAccessor> createAccessor() override;

    virtual void changeRepresentationToBS(const Eigen::Vector& center) override;

    virtual void changeRepresentationToWS() override;

    virtual void setTransform(const Eigen::Affine3d& transform) override;

    // This method is called in the constructor already. It only must be
    // called again, after the face id order of the outer topology was
    // changed. It calls synchronizeTriangleIndexOrder() at the start so that
    // doesn't has to be done.
    //
    // Fixes the outer triangle index order in a way that the following
    // equation is always fulfilled: It is
    // (x_1 - x_0).cross(x_2 - x_0).normalized().dot(x_inner - x_0) < 0
    // where (x_0, x_1, x_2, x_inner) is the single tetrahedron that contains
    // the outer triangle (x_0, x_1, x_2).
    //
    // x_inner is vertex of the tetrahedron that is not part of this outer
    // triangle. It can be part of another one, e.g. a Polygon that is only
    // a single tetrahedron. Then every vertex is part of the outer hull.
    //
    // This equation guarantees that the vertices of an outer triangle are
    // always ordered in a way so that the normal that points outside the
    // tetrahedron (and therefore the polygon) is always:
    // (x_1 - x_0).cross(x_2 - x_0).normalized()
    //
    // The ability to know whats out and inside is important for certain
    // tasks like collision detection.
    //
    // \param printInfo - print the number of fixed triangles indices.
    void fixOuterTriangleIndexOrder(bool printInfo = false);

    // Synchronize indices so that the vertex index order of outer triangles
    // and their global counter part is the same.
    void synchronizeTriangleIndexOrder();

protected:

    bool isInside(ID faceId,
                  const Vector& point,
                  PolygonTopology& topology,
                  BSWSVectors& faceNormals) override;

private:

    std::shared_ptr<Polygon3DData> mData;

    std::shared_ptr<Polygon2DAccessor> mAccessor2D;

    BSWSVectors mOuterVertexNormals;
    BSWSVectors mOuterFaceNormals;

};

#endif // POLYGON3D_H
