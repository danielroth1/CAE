#include "Polygon3D.h"

#include "GeometricDataUtils.h"
#include "Polygon3DDataBS.h"
#include "Polygon3DDataWS.h"
#include "Polygon3DTopology.h"

#include <iomanip>
#include <iostream>
#include <fstream>
#include <map>
#include <scene/data/GeometricDataVisitor.h>
#include <scene/model/ModelUtils.h>
#include <set>
#include <math/MathUtils.h>

Polygon3D::Polygon3D(
        const Vectors& positionsWS,
        const std::shared_ptr<Polygon3DTopology>& topology)
    : Polygon(positionsWS)
{
    mAccessor2D = nullptr;

    // World space construtor
    Polygon::initWorldSpace(positionsWS);
    mData = std::make_shared<Polygon3DDataWS>(topology);

    mOuterVertexNormals.initializeFromWorldSpace(
                GeometricDataUtils::calculateNormals(
                    positionsWS,
                    topology->getOuterFacesIndices3D()));

    // initialize mOuterFaceNormals
    Vectors faceNormals;
    ModelUtils::calculateFaceNormals<double>(
                mPositionData.getPositions(),
                topology->getOuterFacesIndices3D(),
                faceNormals);

    mOuterFaceNormals.initializeFromWorldSpace(faceNormals);

    fixOuterTriangleIndexOrder();
    fixTopology();

    update(true, true, true);
}

Polygon3D::Polygon3D(
        const Vectors& positionsWS,
        const Vectors& vertexNormalsWS,
        const std::shared_ptr<Polygon3DTopology>& topology)
    : Polygon(positionsWS)
{
    mAccessor2D = nullptr;

    // World space construtor
    Polygon::initWorldSpace(positionsWS);
    mData = std::make_shared<Polygon3DDataWS>(topology);

    // intiialize outerVertexNormals correctly?
    mOuterVertexNormals.initializeFromWorldSpace(vertexNormalsWS);

    // initialize mOuterFaceNormals
    Vectors faceNormals;
//    ModelUtils::calculateFaceNormals<double>(
//                mPositionData.getPositions(), faces, faceNormals);
    ModelUtils::calculateFaceNormals<double>(
                mPositionData.getPositions(),
                topology->getOuterFacesIndices3D(),
                faceNormals);

    mOuterFaceNormals.initializeFromWorldSpace(faceNormals);

    fixOuterTriangleIndexOrder();
    fixTopology();

    update(true, true, true);
}

Polygon3D::Polygon3D(
        Vectors* positionsBS,
        const Affine3d& transform,
        const std::shared_ptr<Polygon3DTopology>& topology)
    : Polygon(positionsBS, transform)
{
    mAccessor2D = nullptr;

    Vectors faceNormals = GeometricDataUtils::calculateNormals(
                *positionsBS, topology->getOuterFacesIndices3D());
    std::shared_ptr<Polygon3DDataBS> dataBS =
            std::make_shared<Polygon3DDataBS>(
                topology,
                mPositionData.getPositionsBS(),
                faceNormals,
                ModelUtils::calculateFaceNormals<double>(
                    mPositionData.getPositions(),
                    topology->getOuterFacesIndices3D(),
                    faceNormals));
    mData = dataBS;

    Polygon::initBodySpace(&dataBS->getPositionsBS(), transform);
    mOuterVertexNormals.initializeFromBodySpace(&dataBS->getOuterVertexNormalsBS(),
                                                Eigen::Affine3d(transform.linear()));
    mOuterFaceNormals.initializeFromBodySpace(&dataBS->getOuterFaceNormalsBS(),
                                              Eigen::Affine3d(transform.linear()));

    fixOuterTriangleIndexOrder();
    fixTopology();

    update(true, true, true);
}

Polygon3D::Polygon3D(
        Vectors* positionsBS,
        const Affine3d& transform,
        const Vectors& vertexNormalsBS,
        const std::shared_ptr<Polygon3DTopology>& topology)
    : Polygon (positionsBS, transform)
{
    mAccessor2D = nullptr;

    Vectors faceNormals = GeometricDataUtils::calculateNormals(
                *positionsBS, topology->getOuterFacesIndices3D());
    std::shared_ptr<Polygon3DDataBS> dataBS = std::make_shared<Polygon3DDataBS>(
                topology,
                mPositionData.getPositionsBS(),
                vertexNormalsBS,
                ModelUtils::calculateFaceNormals<double>(
                    mPositionData.getPositions(),
                    topology->getOuterFacesIndices3D(),
                    faceNormals));

    mData = dataBS;

    Polygon::initBodySpace(&dataBS->getPositionsBS(), transform);
    mOuterVertexNormals.initializeFromBodySpace(&dataBS->getOuterVertexNormalsBS(),
                                                Eigen::Affine3d(transform.linear()));
    mOuterFaceNormals.initializeFromBodySpace(&dataBS->getOuterFaceNormalsBS(),
                                              Eigen::Affine3d(transform.linear()));

    fixOuterTriangleIndexOrder();
    fixTopology();

    update(true, true, true);
}

Polygon3D::~Polygon3D()
{

}

Vectors Polygon3D::calcualtePositions2DFrom3D() const
{
    std::vector<unsigned int>& outerVertexIds = mData->getTopology()->getOuterVertexIds();
    Vectors positions2D;
    positions2D.reserve(outerVertexIds.size());
    for (size_t i = 0; i < outerVertexIds.size(); ++i)
    {
        positions2D.push_back(mPositionData.getPositions()[outerVertexIds[i]]);
    }
    return positions2D;
}

std::vector<ID> Polygon3D::retrieveThinCells(double thicknessThreshold) const
{
    std::vector<ID> thinCells;

    std::shared_ptr<Polygon3DTopology> topo3 = mData->getTopology();
    for (const TopologyCell& c : topo3->getCells())
    {
        double thickness = MathUtils::calculateThickness(
                    mPositionData.getPosition(c.getVertexIds()[0]),
                    mPositionData.getPosition(c.getVertexIds()[1]),
                    mPositionData.getPosition(c.getVertexIds()[2]),
                    mPositionData.getPosition(c.getVertexIds()[3]));

        if (thickness < thicknessThreshold)
        {
            thinCells.push_back(c.getID());
        }
    }
    return thinCells;
}

void Polygon3D::removeCells(const std::vector<ID>& cellIds)
{
    if (cellIds.empty())
        return;

    // Process of removing cells:
    // only in the topology:
    // - remove the cells
    // - remove isolated faces
    // - remove isolated edges
    // - remove isolated vertices

    // needs to reinitialize outer faces in the case a cell is removed
    // that had outer faces

    std::shared_ptr<Polygon3DTopology> topo3 = mData->getTopology();

    // remove cell references in other TopologyFeatures
    for (ID cellId : cellIds)
    {
        TopologyCell& cell = topo3->getCells()[cellId];

        for (ID faceId : cell.getFaceIds())
        {
            TopologyFace& face = topo3->getFace(faceId);
            std::vector<ID>& cellIds = face.getCellIds();
            VectorOperations::removeElementInVector(cellIds, cellId);
        }

        for (ID edgeId : cell.getEdgeIds())
        {
            TopologyEdge& edge = topo3->getEdge(edgeId);
            std::vector<ID>& cellIds = edge.getCellIds();
            VectorOperations::removeElementInVector(cellIds, cellId);
        }

        for (ID vertexId : cell.getVertexIds())
        {
            TopologyVertex& vertex = topo3->getVertex(vertexId);
            std::vector<ID>& cellIds = vertex.getCellIds();
            VectorOperations::removeElementInVector(cellIds, cellId);
        }
    }
    VectorOperations::removeVectors(
                topo3->getCells(), cellIds.begin(), cellIds.end());

    // remove isolated faces (faces without cell)
    std::vector<ID> isolatedFaceIds;
    for (TopologyFace& face : topo3->getFaces())
    {
        if (face.getCellIds().empty())
        {
            ID faceId = face.getID();
            for (ID edgeId : face.getEdgeIds())
            {
                TopologyEdge& edge = topo3->getEdge(edgeId);
                std::vector<ID>& faceIds = edge.getFaceIds();
                VectorOperations::removeElementInVector(faceIds, faceId);
            }

            for (ID vertexId : face.getVertexIds())
            {
                TopologyVertex& vertex = topo3->getVertex(vertexId);
                std::vector<ID>& faceIds = vertex.getFaceIds();
                VectorOperations::removeElementInVector(faceIds, faceId);
            }

            isolatedFaceIds.push_back(faceId);
        }
    }
    VectorOperations::removeVectors(
                topo3->getFaces(), isolatedFaceIds.begin(), isolatedFaceIds.end());

    // remove isolated edges (edges without faces)
    std::vector<ID> isolatedEdgeIds;
    for (TopologyEdge& edge : topo3->getEdges())
    {
        if (edge.getFaceIds().empty())
        {
            ID edgeId = edge.getID();
            for (ID vertexId : edge.getVertexIds())
            {
                TopologyVertex& v = topo3->getVertex(vertexId);
                std::vector<ID>& edgeIds = v.getEdgeIds();
                VectorOperations::removeElementInVector(edgeIds, edgeId);
            }

            isolatedEdgeIds.push_back(edgeId);
        }
    }
    VectorOperations::removeVectors(
                topo3->getEdges(), isolatedEdgeIds.begin(), isolatedEdgeIds.end());

    // remove isolated vertices (vertices without edges)
    std::vector<ID> isolatedVertexIds;
    for (TopologyVertex& vertex : topo3->getVertices())
    {
        if (vertex.getEdgeIds().empty())
        {
            isolatedVertexIds.push_back(vertex.getID());
        }
    }
    VectorOperations::removeVectors(
                topo3->getVertices(), isolatedVertexIds.begin(), isolatedVertexIds.end());

    initFromChangedTopology();
}

void Polygon3D::outputToFile(const std::string& filename)
{
    std::ofstream file;

    std::shared_ptr<Polygon3DTopology> topo = mData->getTopology();

    file.open(filename + ".node");
    file << topo->getVertices().size() << "  1\n";

    int vd = static_cast<int>(
                std::ceil(std::log10(static_cast<double>(topo->getVertices().size()))));
    for (size_t i = 0; i < topo->getVertices().size(); ++i)
    {
        const Vector& pos = getPosition(i);
        file << std::setw(vd) << i
             << "   " << pos(0)
             << " " << pos(1)
             << " " << pos(2)
             << "\n";
    }
    file.close();


    {
        file.open(filename + ".ele");
        file << topo->getCells().size() << "  1\n";

        int distance = static_cast<int>(
                    std::ceil(std::log10(static_cast<double>(topo->getCells().size()))));
        for (size_t i = 0; i < topo->getCells().size(); ++i)
        {
            file << std::setw(distance) << i << "  "
                 << std::setw(vd) << topo->getCells()[i].getVertexIds()[0] << " "
                 << std::setw(vd) << topo->getCells()[i].getVertexIds()[1] << " "
                 << std::setw(vd) << topo->getCells()[i].getVertexIds()[2] << " "
                 << std::setw(vd) << topo->getCells()[i].getVertexIds()[3] << " "
                 << "\n";
        }
        file.close();
    }

    {
        file.open(filename + ".face");
        file << topo->getFaces().size() << "  1\n";

        int distance = static_cast<int>(
                    std::ceil(std::log10(static_cast<double>(topo->getFaces().size()))));
        for (size_t i = 0; i < topo->getFaces().size(); ++i)
        {
            file << std::setw(distance) << i << "  "
                 << std::setw(vd) << topo->getFaces()[i].getVertexIds()[0] << " "
                 << std::setw(vd) << topo->getFaces()[i].getVertexIds()[1] << " "
                 << std::setw(vd) << topo->getFaces()[i].getVertexIds()[2] << " "
                 << std::setw(vd) << "-1"
                 << "\n";
        }
        file.close();
    }
}

Polygon3DTopology& Polygon3D::getTopology3D()
{
    return *mData->getTopology().get();
}

const Polygon3DTopology& Polygon3D::getTopology3D() const
{
    return *mData->getTopology().get();
}

std::vector<unsigned int>& Polygon3D::getOuterPositionIds()
{
    return mData->getTopology()->getOuterVertexIds();
}

Vectors& Polygon3D::getOuterVertexNormals()
{
    return mOuterVertexNormals.getVectors();
}

Vectors& Polygon3D::getOuterFaceNormals()
{
    return mOuterFaceNormals.getVectors();
}

std::shared_ptr<Polygon3DData> Polygon3D::getData3D()
{
    return mData;
}

void Polygon3D::updateBoundingBox()
{
    GeometricDataUtils::updateBoundingBox(
                mPositionData.getPositions(), mBoundingBox);
}

void Polygon3D::accept(GeometricDataVisitor& visitor)
{
    visitor.visit(*this);
}

void Polygon3D::update(bool updateFaceNormals,
                       bool updateVertexNormals,
                       bool notifyListeners)
{
    Polygon::update(updateFaceNormals, updateVertexNormals, notifyListeners);

    if (mOuterVertexNormals.getType() == BSWSVectors::Type::BODY_SPACE)
    {
        if (updateVertexNormals)
            mOuterVertexNormals.update();

        if (updateFaceNormals)
            mOuterFaceNormals.update();
    }
    else
    {
        if (updateVertexNormals)
        {
            ModelUtils::calculateNormals<double>(
                        calcualtePositions2DFrom3D(),
    //                    mPositionData.getPositions(),
                        mData->getTopology()->getOuterTopology().getFacesIndices(),
                        mOuterVertexNormals.getVectors());
        }

        if (updateFaceNormals)
        {
//            ModelUtils::calculateFaceNormals<double>(
//                        calcualtePositions2DFrom3D(),
//    //                    mPositionData.getPositions(),
//                        mData->getTopology()->getOuterTopology().getFacesIndices(),
//                        mOuterFaceNormals.getVectors());

//            getAccessor2D();

            // for some reason the accessor is corrupted here.

            std::vector<Vector>& normals = mOuterFaceNormals.getVectors();
            std::vector<Face>& faces =
                    mData->getTopology()->getOuterTopology().getFacesIndices();

            normals.resize(faces.size());
            for (size_t i = 0; i < faces.size(); ++i)
            {
                const Face& t = faces[i];

                const Eigen::Vector& p0 = getPosition(mData->getTopology()->to3DVertexIndex(t[0]));
                const Eigen::Vector& p1 = getPosition(mData->getTopology()->to3DVertexIndex(t[1]));
                const Eigen::Vector& p2 = getPosition(mData->getTopology()->to3DVertexIndex(t[2]));

                normals[i] = (p0 - p1).cross(p0 - p2).normalized();
            }
        }
    }
}

void Polygon3D::fixTopology()
{
    std::set<ID> notReferenced;

    std::vector<ID> notReferencedByEdges =
            mData->getTopology()->retrieveNotReferencedByEdges();
    notReferenced.insert(notReferencedByEdges.begin(),
                         notReferencedByEdges.end());

    std::vector<ID> notReferencedByFaces =
            mData->getTopology()->retrieveNotReferencedByFaces();
    notReferenced.insert(notReferencedByFaces.begin(),
                         notReferencedByFaces.end());

    std::vector<ID> notReferencedByCells =
            mData->getTopology()->retrieveNotReferencedByCells();
    notReferenced.insert(notReferencedByCells.begin(),
                         notReferencedByCells.end());


    if (notReferencedByEdges.size() > 0)
        std::cout << "remove " << notReferencedByEdges.size() << " vertices that "
                  << "are not referenced by edges.\n";

    if (notReferencedByFaces.size() > 0)
        std::cout << "remove " << notReferencedByFaces.size() << " vertices that "
                  << "are not referenced by faces.\n";

    if (notReferencedByCells.size() > 0)
        std::cout << "remove " << notReferencedByCells.size() << " vertices that "
                  << "are not referenced by cells.\n";

    std::vector<ID> notReferencedVector;
    notReferencedVector.insert(notReferencedVector.end(),
                         notReferenced.begin(),
                         notReferenced.end());

    // Should not be necessary becaues elements are already sorted in
    // set and vector is sorted anyways in remove method.
    std::sort(notReferencedVector.begin(), notReferencedVector.end());

    if (notReferencedVector.size() > 0)
        std::cout << "remove a total of " << notReferencedVector.size()
                  << " vertices\n";

    removeVertices(notReferencedVector);

    removeCells(retrieveThinCells(1e-5));

    fixOuterTriangleIndexOrder();
}

void Polygon3D::removeVertex(ID index)
{
    Polygon::removeVertex(index);
    mData->removeVector(index);
    mOuterFaceNormals.removeVector(index);
    mOuterVertexNormals.removeVector(index);
}

void Polygon3D::removeVertices(std::vector<ID>& indices)
{
    if (indices.empty())
        return;

    Polygon::removeVertices(indices);
    mData->removeVectors(indices);

    // TODO: this doesn't work because outer elements have different
    // indices. Also removing inner vertices, doesn't affect them at all.
    mOuterFaceNormals.removeVectors(indices.begin(), indices.end());
    mOuterVertexNormals.removeVectors(indices.begin(), indices.end());
}

bool Polygon3D::isInside(const TopologyFeature& feature, Vector point)
{
    return Polygon::isInside(
                feature,
                point,
                mData->getTopology()->getOuterTopology(),
                mOuterFaceNormals);
}

bool Polygon3D::isInside(
        const TopologyFeature& feature,
        Vector source,
        double distance,
        Vector target)
{
    return Polygon::isInside(
                feature, source, distance, target,
                mData->getTopology()->getOuterTopology(), mOuterFaceNormals);
}

Polygon::DimensionType Polygon3D::getDimensionType() const
{
    return DimensionType::THREE_D;
}

std::shared_ptr<PolygonData> Polygon3D::getData()
{
    return mData;
}

PolygonTopology& Polygon3D::getTopology()
{
    return *mData->getTopology().get();
}

const std::shared_ptr<Polygon2DAccessor>& Polygon3D::getAccessor2D()
{
    if (!mAccessor2D)
        mAccessor2D = createAccessor();
    return mAccessor2D;
}

std::shared_ptr<Polygon2DAccessor> Polygon3D::createAccessor()
{
    class OuterPolygon2DAccessor : public Polygon2DAccessor
    {
    public:
        OuterPolygon2DAccessor(Polygon3D* _poly3)
            : poly3(_poly3)
        {

        }

        virtual ~OuterPolygon2DAccessor() override
        {

        }

        virtual bool isInside(
                const TopologyFeature& feature, const Eigen::Vector& source,
                double distance, const Eigen::Vector& target) override
        {
            return poly3->isInside(feature, source, distance, target);
        }

        virtual Polygon* getPolygon() const override
        {
            return poly3;
        }

        virtual size_t getSize() override
        {
            return poly3->getOuterPositionIds().size();
        }

        virtual void setPosition(size_t index, const Eigen::Vector& position) override
        {
            poly3->setPosition(poly3->mData->getTopology()->to3DVertexIndex(index), position);
        }

        virtual Eigen::Vector& getPosition(std::size_t index) override
        {
            return poly3->getPosition(poly3->mData->getTopology()->to3DVertexIndex(index));
        }

        virtual Polygon2DTopology& getTopology2D() override
        {
            return poly3->mData->getTopology()->getOuterTopology();
        }

        virtual const Polygon2DTopology& getTopology2D() const override
        {
            return poly3->mData->getTopology()->getOuterTopology();
        }

        virtual Vectors& getVertexNormals() override
        {
            return poly3->mOuterVertexNormals.getVectors();
        }

        virtual Vectors& getFaceNormals() override
        {
            return poly3->mOuterFaceNormals.getVectors();
        }

    private:
        Polygon3D* poly3;
    };

    return std::make_shared<OuterPolygon2DAccessor>(this);
}

void Polygon3D::changeRepresentationToBS(const Vector& center)
{
    std::shared_ptr<Polygon3DDataWS> dataWS =
            std::dynamic_pointer_cast<Polygon3DDataWS>(mData);
    if (dataWS)
    {
        std::shared_ptr<Polygon3DDataBS> dataBS =
                std::make_shared<Polygon3DDataBS>(
                    mData->getTopology(),
                    mPositionData.getPositions(),
                    mOuterVertexNormals.getVectors(),
                    mOuterFaceNormals.getVectors());
        mData = dataBS;

        mPositionData.changeRepresentationToBS(
                    &dataBS->getPositionsBS(), Eigen::Affine3d::Identity());

        mPositionData.moveCenterTo(center);
        mOuterVertexNormals.changeRepresentationToBS(
                    &dataBS->getOuterVertexNormalsBS(), Eigen::Affine3d::Identity());
        mOuterFaceNormals.changeRepresentationToBS(
                    &dataBS->getOuterFaceNormalsBS(), Eigen::Affine3d::Identity());
    }
}

void Polygon3D::changeRepresentationToWS()
{
    std::shared_ptr<Polygon3DDataBS> dataBS =
            std::dynamic_pointer_cast<Polygon3DDataBS>(mData);
    if (dataBS)
    {
        std::shared_ptr<Polygon3DDataWS> dataWS =
                std::make_shared<Polygon3DDataWS>(mData->getTopology());
        mData = dataWS;

        mPositionData.changeRepresentationToWS();
        mOuterVertexNormals.changeRepresentationToWS();
        mOuterFaceNormals.changeRepresentationToWS();
    }
}

void Polygon3D::setTransform(const Affine3d& transform)
{
    Polygon::setTransform(transform);

    Affine3d linear(transform.linear());
    mOuterVertexNormals.setTransform(linear);
    mOuterFaceNormals.setTransform(linear);
}

void Polygon3D::fixOuterTriangleIndexOrder(bool printInfo)
{
    printInfo = true;
    std::shared_ptr<Polygon3DTopology> topology = mData->getTopology();

    synchronizeTriangleIndexOrder();

    int fixedFacesCounter = 0;

    int counter = -1;
    int differenceCounter = 0;
    int noDifferenceCounter = 0;
    int volumeZeroCounter = 0;

    for (TopologyFace& outerFace2D : topology->getOuterTopology().getFaces())
    {
        ++counter;
        ID face3DId = topology->to3DFaceIndex(outerFace2D.getID());
        TopologyFace& face3D = topology->getFace(face3DId);

        if (face3D.getCellIds().empty())
            continue;

        if (face3D.getCellIds().size() != 1)
            std::cout << "error, outer face has multiple cells.\n";
        TopologyCell& cell = topology->getCells()[face3D.getCellIds()[0]];
        Cell& c = cell.getVertexIds();
        Face& f = face3D.getVertexIds();

        // find the id of the (inner) vertex that is part of the tetrahedron
        // but not the outer triangle.
        ID innerVertexId;
        bool found = false;
        for (size_t i = 0; i < 4; ++i)
        {
            if (std::find(f.begin(), f.end(), c[i]) == f.end())
            {
                innerVertexId = c[i];
                found = true;
                break;
            }
        }
        if (!found)
        {
            std::cerr << "Error: detected cell with duplicated indices.\n";
            continue;
        }

        Vector innerPoint = mPositionData.getPosition(innerVertexId);

        Vector r1 = mPositionData.getPosition(f[1]) -
                mPositionData.getPosition(f[0]);
        Vector r2 = mPositionData.getPosition(f[2]) -
                mPositionData.getPosition(f[0]);
        Vector r3 = innerPoint - mPositionData.getPosition(f[0]);

        // Reverts the face indices so that its normal:
        // (x2 - x0).cross(x1 - x0).normalized()
        // points in the other direction.
        auto revertFace = [](Face& f)
        {
            unsigned int temp = f[1];
            f[1] = f[2];
            f[2] = temp;
        };

        double result = r1.cross(r2).normalized().dot(r3);

        if (std::abs(result) < 1e-5)
        {
            ++volumeZeroCounter;

            // volume of tetrahedron is near zero, so its basically
            // a triangle. In that case numerical errors can
            // lead to normals being calculated wrong.
            // The idea is to calculate the normale w.r.t.
            // a neighbored tetrahedron.

            // Get the id of a neighbored face
            TopologyCell& cell = topology->getCells()[face3D.getCellIds()[0]];
            ID innerFaceId = cell.getFaceIds()[0];
            if (innerFaceId == face3D.getID())
                innerFaceId = cell.getFaceIds()[1];
            TopologyFace& innerFace = topology->getFace(innerFaceId);

            ID neighboredCellId = innerFace.getCellIds()[0];
            if (neighboredCellId == cell.getID() && innerFace.getCellIds().size() > 1)
            {
                neighboredCellId = innerFace.getCellIds()[1];
            }

            if (neighboredCellId != cell.getID())
            {
                TopologyCell& neighboredCell = topology->getCells()[neighboredCellId];

                ID otherInnerVertexId = neighboredCell.getVertexIds()[0];
                for (size_t i = 1; i < 4; ++i)
                {
                    ID candidatedId = neighboredCell.getVertexIds()[i];
                    Cell& cellVertexIds = cell.getVertexIds();
                    if (std::find(cellVertexIds.begin(), cellVertexIds.end(),
                                  candidatedId) == cellVertexIds.end())
                    {
                        otherInnerVertexId = candidatedId;
                        break;
                    }
                }

                innerPoint = mPositionData.getPosition(otherInnerVertexId);
                Vector r1 = mPositionData.getPosition(f[1]) -
                        mPositionData.getPosition(f[0]);
                Vector r2 = mPositionData.getPosition(f[2]) -
                        mPositionData.getPosition(f[0]);
                Vector r3 = innerPoint -
                        mPositionData.getPosition(f[0]);

                double result2 = r1.cross(r2).normalized().dot(r3);

                if ((result < 0 && result2 > 0) ||
                        (result > 0 && result2 < 0))
                {
                    differenceCounter++;
                }
                else
                {
                    noDifferenceCounter++;
                }

                result = result2;

                if (std::abs(result) < 1e-5)
                {
                    std::cout << "Normal calculation for outer "
                              << "face with id " << face3D.getID()
                              << " might have gone wrong.\n";
                }

            }
        }

        if (result > 0)
        {
            // revert the following redundant stored faces
            // Polyogn3DTopology:
            // topology->getFaces()
            // topology->getFacesIndices()
            // topology->getOuterFacesIndices3D()
            // Polygon2DTopology:
            // topology->getOuterTopology().getFaces()
            // topology->getOuterTopology().getFacesIndices()

            // topology
            revertFace(f);
            revertFace(topology->getFacesIndices()[face3DId]);
            revertFace(topology->getOuterFacesIndices3D()[outerFace2D.getID()]);

            // outer topology
            revertFace(outerFace2D.getVertexIds());
            revertFace(topology->getOuterTopology().getFacesIndices()[outerFace2D.getID()]);

            topology->setFaceOwnerships();
            topology->getOuterTopology().setFaceOwnerships();

            ++fixedFacesCounter;
        }
    }

    std::cout << "VOLUME ZEROS  = " << volumeZeroCounter << "\n";
    std::cout << "DIFFERENCE    = " << differenceCounter << "\n";
    std::cout << "NO DIFFERENCE = " << noDifferenceCounter << "\n";

    if (printInfo)
        std::cout << "Fixed " << fixedFacesCounter << " faces.\n";
}

void Polygon3D::synchronizeTriangleIndexOrder()
{
    std::shared_ptr<Polygon3DTopology> topology = mData->getTopology();

    for (TopologyFace& outerFace2D : topology->getOuterTopology().getFaces())
    {
        ID face3DId = topology->to3DFaceIndex(outerFace2D.getID());
        TopologyFace& face3D = topology->getFace(face3DId);

        for (size_t i = 0; i < 3; ++i)
        {
            unsigned int index3d =
                    static_cast<unsigned int>(
                        topology->to3DVertexIndex(outerFace2D.getVertexIds()[i]));
            face3D.getVertexIds()[i] = index3d;
            topology->getFacesIndices()[face3DId][i] = index3d;
            topology->getOuterFacesIndices3D()[outerFace2D.getID()][i] = index3d;
        }
    }
}

std::tuple<size_t, double, Vector4d> Polygon3D::findTetrahedron(
        const Vector3d& p)
{
    // Find all cells that contain the vertex (can be multiple if the vertex
    // lies between two cells within the margin wt).

    std::tuple<size_t, double, Eigen::Vector4d> tuple;
    Eigen::Vector4d& bary = std::get<2>(tuple);

    for (size_t j = 0; j < getTopology3D().getCells().size(); ++j)
    {
        Cell& c = getTopology3D().getCellIds()[j];

        MathUtils::projectPointOnTetrahedron(
                    mPositionData.getPosition(c[0]),
                mPositionData.getPosition(c[1]),
                mPositionData.getPosition(c[2]),
                mPositionData.getPosition(c[3]),
                p,
                bary);

        if (0 < bary[0] && bary[0] < 1 &&
            0 < bary[1] && bary[1] < 1 &&
            0 < bary[2] && bary[2] < 1 &&
            0 < bary[3] && bary[3] < 1)
        {
            std::get<0>(tuple) = j;
            std::get<1>(tuple) = 0.0;
            return tuple;
        }
    }

    // The point lies outside. Search for the closests outside triangle.
    // Then calculate the baryzentric coordinates for the corresponding
    // tetrahedron and use those.
    // Visual artifacts are limitted if the point is close enough to
    // the tetrahedron.

    // obtain index of closest triangle:
    // triangle_index, distance
    std::vector<std::tuple<size_t, double>> tuples;

    std::vector<TopologyFace>& outerFaces =
            getTopology3D().getOuterTopology().getFaces();
    for (size_t j = 0; j < outerFaces.size(); ++j)
    {
        // If this outer triangle is not bound to a cell, ignore it.
        Polygon3DTopology& pt = getTopology3D();
        ID f3D = pt.to3DFaceIndex(j);
        TopologyFace& f3d = pt.getFace(f3D);
        if (f3d.getCellIds().empty())
            continue;

        std::array<Vector, 3> v; // vertices
        v[0] = getPosition(f3d.getVertexIds()[0]);
        v[1] = getPosition(f3d.getVertexIds()[1]);
        v[2] = getPosition(f3d.getVertexIds()[2]);

        Eigen::Vector3d inter;
        bool isInside;
        Eigen::Vector3d bary3;
        MathUtils::projectPointOnTriangle(v[0], v[1], v[2], p, inter, bary3, isInside);

        double distance = (inter - p).norm();
        tuples.push_back(std::make_tuple(j, distance));
    }

    std::sort(tuples.begin(), tuples.end(),
              [](const std::tuple<size_t, double>& t1,
              const std::tuple<size_t, double>& t2)
    {
        return std::get<1>(t1) < std::get<1>(t2);
    });

    if (!tuples.empty())
    {
        size_t closestTriangleIndex = std::get<0>(tuples[0]);
        double distance = std::get<1>(tuples[0]);

        // We have a face of the outer face mesh. First, we need to
        // convert it to the one of the corresponding inner mesh.
        Polygon3DTopology& pt = getTopology3D();
        ID f3D = pt.to3DFaceIndex(closestTriangleIndex);
        TopologyFace& f = pt.getFace(f3D);

        if (f.getCellIds().empty())
        {
            std::cerr << "Error: No adjacent cell found for outer face.\n";
        }
        else
        {
            if (f.getCellIds().size() > 1)
            {
                std::cout << "Warning: Outer face has multiple adjacent "
                             "cells. One cell is chosen arbitrarily.\n";
            }

            ID cellId = f.getCellIds()[0];
            Cell& c = getTopology3D().getCellIds()[cellId];

            // Sets std::get<2>(tuple) [the barycentric coordinates]
            MathUtils::projectPointOnTetrahedron(
                        mPositionData.getPosition(c[0]),
                    mPositionData.getPosition(c[1]),
                    mPositionData.getPosition(c[2]),
                    mPositionData.getPosition(c[3]),
                    p,
                    bary);

            std::get<0>(tuple) = closestTriangleIndex;
            std::get<1>(tuple) = distance;
            return tuple;
        }
    }
}

bool Polygon3D::isInside(ID faceId, const Vector& point,
                         PolygonTopology& topology, BSWSVectors& faceNormals)
{
    ID vertexId = topology.getFace(faceId).getVertexIds()[0];
    return faceNormals.getVector(faceId)
            .dot(point - mPositionData.getPosition(mData->getTopology()->getOuterVertexIds()[vertexId])) < 0;
}

void Polygon3D::initFromChangedTopology()
{
    // Inits the topology only from the TopologyFeatures

    std::shared_ptr<Polygon3DTopology> topo3 = mData->getTopology();

    // create mapping for all types of ids
    // they map real index -> stored index and can be used to bring
    // any type of element that references the old data structures in sync.
    std::map<ID, ID> vertexMapping;
    std::map<ID, ID> edgeMapping;
    std::map<ID, ID> faceMapping;
    std::map<ID, ID> cellMapping;

    // cell mapping should be different, not i -> i

    // vertices = (0, 1, 2, 4, 5, 8, 9)
    // -> removed vertices (3, 6, 7)
    // vertexMapping: (0, 0), (1, 1), (2, 2), (4, 3), (5, 4), (8, 5), (9, 6)

    // points from the old id to the new one (after everything is removed)

    // convert old face (0, 5, 8) to (0, 4, 5)
    //
    //
    // Requirement of this function:
    // there is no more topological element referencing an element that
    // doesn't exist anymore.
    // -> topologies ids are unchanged
    //   -> it uses the old topology vectors to reference topology elements
    // -> despite that, topology is consistent
    //

    // Init vertex mapping
    for (size_t i = 0; i < topo3->getVertices().size(); ++i)
    {
        vertexMapping[topo3->getVertices()[i].getID()] = i;
    }

    // Init edge mapping
    for (size_t i = 0; i < topo3->getEdges().size(); ++i)
    {
        edgeMapping[topo3->getEdges()[i].getID()] = i;
    }

    // Init face mapping
    for (size_t i = 0; i < topo3->getFaces().size(); ++i)
    {
        faceMapping[topo3->getFaces()[i].getID()] = i;
    }

    // Init cell mapping
    for (size_t i = 0; i < topo3->getCells().size(); ++i)
    {
        cellMapping[topo3->getCells()[i].getID()] = i;
    }

    for (size_t i = 0; i < topo3->getVertices().size(); ++i)
    {
        topo3->getVertices()[i].setID(i);
    }

    for (size_t i = 0; i < topo3->getEdges().size(); ++i)
    {
        topo3->getEdges()[i].setID(i);
    }

    for (size_t i = 0; i < topo3->getFaces().size(); ++i)
    {
        topo3->getFaces()[i].setID(i);
    }

    for (size_t i = 0; i < topo3->getCells().size(); ++i)
    {
        topo3->getCells()[i].setID(i);
    }

    // Apply the mapping to all ids of all the other features


    // Probelem: mapping is applied to vertices / edges / faces / cells
    // that were removed. This fills the mapping with zero entries.
    // Also, these entries should be removed eventually.

    // Apply mappings

    for (TopologyCell& cell : topo3->getCells())
    {
        // Cells
        for (size_t i = 0; i < cell.getFaceIds().size(); ++i)
        {
            // TODO: doen't make sense because a cell always has 4 faces
            auto it = faceMapping.find(cell.getFaceIds()[i]);
            if (it == faceMapping.end())
                cell.getFaceIds().erase(cell.getFaceIds().begin() + i);
            else
                cell.getFaceIds()[i] = faceMapping[cell.getFaceIds()[i]];
        }

        // Edge
        for (size_t i = 0; i < cell.getEdgeIds().size(); ++i)
        {
            cell.getEdgeIds()[i] = edgeMapping[cell.getEdgeIds()[i]];
        }

        // Vertex
        for (size_t i = 0; i < cell.getVertexIds().size(); ++i)
        {
            cell.getVertexIds()[i] = vertexMapping[cell.getVertexIds()[i]];
        }
    }

    for (TopologyFace& face : topo3->getFaces())
    {
        // Cells
        for (size_t i = 0; i < face.getCellIds().size(); ++i)
        {
            auto it = cellMapping.find(face.getCellIds()[i]);
            if (it == cellMapping.end())
            {
                face.getCellIds().erase(face.getCellIds().begin() + i);
            }
            else
                face.getCellIds()[i] = cellMapping[face.getCellIds()[i]];
        }

        // Edge
        for (size_t i = 0; i < face.getEdgeIds().size(); ++i)
        {
            face.getEdgeIds()[i] = edgeMapping[face.getEdgeIds()[i]];
        }

        // Vertex
        for (size_t i = 0; i < face.getVertexIds().size(); ++i)
        {
            face.getVertexIds()[i] = vertexMapping[face.getVertexIds()[i]];
        }
    }

    for (TopologyEdge& edge : topo3->getEdges())
    {
        // Cell
        for (size_t i = 0; i < edge.getCellIds().size(); ++i)
        {
            auto it = cellMapping.find(edge.getCellIds()[i]);
            if (it == cellMapping.end())
                edge.getCellIds().erase(edge.getCellIds().begin() + i);
            else
                edge.getCellIds()[i] = cellMapping[edge.getCellIds()[i]];
        }

        // Face
        for (size_t i = 0; i < edge.getFaceIds().size(); ++i)
        {
            auto it = faceMapping.find(edge.getFaceIds()[i]);
            if (it == faceMapping.end())
                edge.getFaceIds().erase(edge.getCellIds().begin() + i);
            else
                edge.getFaceIds()[i] = it->second;
        }

        // Vertex
        for (size_t i = 0; i < edge.getVertexIds().size(); ++i)
        {
            edge.getVertexIds()[i] = vertexMapping[edge.getVertexIds()[i]];
        }
    }

    for (TopologyVertex& vertex : topo3->getVertices())
    {
        // Cell
        for (size_t i = 0; i < vertex.getCellIds().size(); ++i)
        {
            auto it = cellMapping.find(vertex.getCellIds()[i]);
            if (it == cellMapping.end())
                vertex.getCellIds().erase(vertex.getCellIds().begin() + i);
            else
                vertex.getCellIds()[i] = it->second;
        }

        // Face
        for (size_t i = 0; i < vertex.getFaceIds().size(); ++i)
        {
            auto it = faceMapping.find(vertex.getFaceIds()[i]);
            if (it == faceMapping.end())
                vertex.getFaceIds().erase(vertex.getCellIds().begin() + i);
            else
                vertex.getFaceIds()[i] = it->second;
        }

        // Edge
        for (size_t i = 0; i < vertex.getEdgeIds().size(); ++i)
        {
            auto it = edgeMapping.find(vertex.getEdgeIds()[i]);
            if (it == edgeMapping.end())
                vertex.getEdgeIds().erase(vertex.getEdgeIds().begin() + i);
            else
                vertex.getEdgeIds()[i] = it->second;
        }
    }

    // Result: a consistent Topology
    // Next: use the mapping to update all the auxilary data structures

    std::vector<ID> removeVertices = retrieveRemovableIndices(vertexMapping);
    std::vector<ID> removeEdges = retrieveRemovableIndices(edgeMapping);
    std::vector<ID> removeFaces = retrieveRemovableIndices(faceMapping);
    std::vector<ID> removeCells = retrieveRemovableIndices(cellMapping);

    // change mapping:
    // converts the indices from old to new

//    changeMapping(vertexMapping, mOuterVertexNormals.getVectors());
//    changeMapping(faceMapping, mOuterFaceNormals.getVectors());

    // Vectors
    // check for all removed ids if they were part of outer topology.
    // If so, add them to remove***Outer
    // Convert their indices so that they point to elements of the 3d topology.

    auto toOuter = [](
            const std::vector<ID>& removeVertices, // global indices
            const std::vector<unsigned int>& outerVertexIds)
    {
        std::vector<ID> removeElementsOut;

        for (ID removeVertexId : removeVertices)
        {
            auto it = std::find(outerVertexIds.begin(),
                                outerVertexIds.end(),
                                removeVertexId);

            if (it != outerVertexIds.end())
            {
                size_t index = it - outerVertexIds.begin();
                removeElementsOut.push_back(index);
            }
        }

        return removeElementsOut;
    };


    std::vector<ID> removeVerticesOuter =
            toOuter(removeVertices, topo3->getOuterVertexIds());
    std::vector<ID> removeEdgesOuter =
            toOuter(removeEdges, topo3->getOuterEdgeIds());
    std::vector<ID> removeFacesOuter =
            toOuter(removeFaces, topo3->getOuterFaceIds());

    // Adapt all the other data structures that reference removed features.
    VectorOperations::removeVectors(topo3->getOuterVertexIds(), removeVerticesOuter);
    VectorOperations::removeVectors(mOuterVertexNormals.getVectors(), removeVertices);

    VectorOperations::removeVectors(topo3->getOuterEdgeIds(), removeEdgesOuter);

    VectorOperations::removeVectors(topo3->getFacesIndices(), removeFaces);
    VectorOperations::removeVectors(topo3->getOuterFaceIds(), removeFacesOuter);
    VectorOperations::removeVectors(topo3->getOuterFacesIndices3D(), removeFacesOuter);
    VectorOperations::removeVectors(mOuterFaceNormals.getVectors(), removeFacesOuter);

    VectorOperations::removeVectors(topo3->getCellIds(), removeCells);

    // Sync outer topology feature ids
    for (size_t i = 0; i < topo3->getOuterVertexIds().size(); ++i)
    {
        topo3->getOuterVertexIds()[i] =  vertexMapping[topo3->getOuterVertexIds()[i]];
    }

    for (size_t i = 0; i < topo3->getOuterEdgeIds().size(); ++i)
    {
        topo3->getOuterEdgeIds()[i] =  edgeMapping[topo3->getOuterEdgeIds()[i]];
    }

    for (size_t i = 0; i < topo3->getOuterFaceIds().size(); ++i)
    {
        topo3->getOuterFaceIds()[i] =  faceMapping[topo3->getOuterFaceIds()[i]];
    }

    // add new outer faces
    std::vector<ID> potentialOuterVertices;
    int addedOuterFaces = 0;
    for (TopologyFace& face : topo3->getFaces())
    {
        if (face.getCellIds().size() == 1)
        {
            if (std::find(topo3->getOuterFaceIds().begin(),
                          topo3->getOuterFaceIds().end(), face.getID())
                    == topo3->getOuterFaceIds().end())
            {
                topo3->getOuterFacesIndices3D().push_back(face.getVertexIds());
                topo3->getOuterFaceIds().push_back(face.getID());
                ++addedOuterFaces;
                for (size_t i = 0; i < 3; ++i)
                {
                    potentialOuterVertices.push_back(face.getVertexIds()[i]);
                }
            }
        }
    }
    std::cout << "Added outer face: " << addedOuterFaces << ".\n";

    // add potential new outer vertices
    // Create a copy of current outer vertex ids.
    std::vector<unsigned int> outerIds = topo3->getOuterVertexIds();
    for (ID vId : potentialOuterVertices)
    {
        if (std::find(outerIds.begin(), outerIds.end(), vId) == outerIds.end())
        {
            topo3->getOuterVertexIds().push_back(vId);
        }
    }
    std::cout << "Added outer vertices: " << topo3->getOuterVertexIds().size() - outerIds.size() << ".\n";

    topo3->init();
}

void Polygon3D::changeMapping(const std::map<ID, ID>& mapping, Vectors& vectors)
{
    for (const std::pair<ID, ID>& pair : mapping)
    {
        vectors[pair.first] = vectors[pair.second];
    }
}

std::vector<ID> Polygon3D::retrieveRemovableIndices(const std::map<ID, ID>& mapping)
{
    std::vector<ID> removeIndices;
    size_t index = 0;
    for (const std::pair<ID, ID>& pair : mapping)
    {
        while (pair.first != index)
        {
            removeIndices.push_back(index);
            ++index;
        }
        ++index;
    }
    return removeIndices;
}

bool Polygon3D::checkFaceNormalDirection()
{
    std::vector<Vector>& normals = mOuterFaceNormals.getVectors();
    std::vector<Face>& faces =
            mData->getTopology()->getOuterTopology().getFacesIndices();

    std::shared_ptr<Polygon3DTopology> topo3 = mData->getTopology();
    normals.resize(faces.size());

    bool correct = true;
    for (size_t i = 0; i < faces.size(); ++i)
    {
        TopologyFace& face3D = mData->getTopology()->getFace(mData->getTopology()->getOuterFaceIds()[i]);

        Face& face3Dids = face3D.getVertexIds();
        ID otherID;
        for (size_t j = 0; j < 4; ++j)
        {
            mData->getTopology()->getCells()[face3D.getCellIds()[0]].getVertexIds()[j];
            ID cellId = mData->getTopology()->getCells()[face3D.getCellIds()[0]].getVertexIds()[j];
            if (std::find(face3Dids.begin(), face3Dids.end(), cellId) == face3Dids.end())
            {
                otherID = cellId;
                break;
            }
        }
        Vector point = mPositionData.getPosition(otherID);

        Vector r1 = mPositionData.getPosition(face3Dids[1]) -
                mPositionData.getPosition(face3Dids[0]);
        Vector r2 = mPositionData.getPosition(face3Dids[2]) -
                mPositionData.getPosition(face3Dids[0]);
        Vector r3 = point -
                mPositionData.getPosition(face3Dids[0]);

        double result = r1.cross(r2).normalized().dot(r3);

        if (std::abs(result) < 1e-8)
        {
            // volume of tetrahedron is near zero, so its basically
            // a triangle. In that case numerical errors can
            // lead to normals being calculated wrong.
            // The idea is to calculate the normale w.r.t.
            // a neighbored tetrahedron.

            // Get the id of a neighbored face
            TopologyCell& cell = topo3->getCells()[face3D.getCellIds()[0]];
            ID innerFaceId = cell.getFaceIds()[0];
            if (innerFaceId == face3D.getID())
                innerFaceId = cell.getFaceIds()[1];
            TopologyFace& innerFace = topo3->getFace(innerFaceId);

            ID neighboredCellId = innerFace.getCellIds()[0];
            if (neighboredCellId == cell.getID() && innerFace.getCellIds().size() > 1)
            {
                neighboredCellId = innerFace.getCellIds()[1];
                TopologyCell& neighboredCell = topo3->getCells()[neighboredCellId];

                ID otherInnerVertexId = neighboredCell.getVertexIds()[0];
                for (size_t i = 1; i < 4; ++i)
                {
                    ID candidatedId = neighboredCell.getVertexIds()[i];
                    if (std::find(face3Dids.begin(), face3Dids.end(),
                                  candidatedId) == face3Dids.end())
                    {
                        otherInnerVertexId = candidatedId;
                        break;
                    }
                }

                point = mPositionData.getPosition(otherInnerVertexId);
                Vector r1 = mPositionData.getPosition(face3Dids[1]) -
                        mPositionData.getPosition(face3Dids[0]);
                Vector r2 = mPositionData.getPosition(face3Dids[2]) -
                        mPositionData.getPosition(face3Dids[0]);
                Vector r3 = point -
                        mPositionData.getPosition(face3Dids[0]);

                double result = r1.cross(r2).normalized().dot(r3);

                if (std::abs(result) < 1e-8)
                {
                    correct = false;
                    std::cout << "Normal calculation for outer "
                              << "face with id " << face3D.getID()
                              << " might have gone wrong.\n";
                }
            }

        }
        if (result > 0)
            std::cout << "error 1: normal is facing wrong direction: " << result << "\n";

    }
    return correct;
}
