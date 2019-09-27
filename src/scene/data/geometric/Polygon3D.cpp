#include "Polygon3D.h"

#include "GeometricDataUtils.h"
#include "Polygon3DDataBS.h"
#include "Polygon3DDataWS.h"
#include "Polygon3DTopology.h"

#include <iostream>
#include <scene/data/GeometricDataVisitor.h>
#include <scene/model/ModelUtils.h>
#include <set>

Polygon3D::Polygon3D(
        const Vectors& positionsWS,
        const std::shared_ptr<Polygon3DTopology>& topology)
    : Polygon(positionsWS)
{
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
}

Polygon3D::Polygon3D(
        const Vectors& positionsWS,
        const Vectors& vertexNormalsWS,
        const std::shared_ptr<Polygon3DTopology>& topology)
    : Polygon(positionsWS)
{
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
}

Polygon3D::Polygon3D(
        Vectors* positionsBS,
        const Affine3d& transform,
        const std::shared_ptr<Polygon3DTopology>& topology)
    : Polygon(positionsBS, transform)
{
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
}

Polygon3D::Polygon3D(
        Vectors* positionsBS,
        const Affine3d& transform,
        const Vectors& vertexNormalsBS,
        const std::shared_ptr<Polygon3DTopology>& topology)
    : Polygon (positionsBS, transform)
{
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

void Polygon3D::update(bool updateFaceNormals, bool updateVertexNormals)
{
    Polygon::update(updateFaceNormals, updateVertexNormals);

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
            ModelUtils::calculateFaceNormals<double>(
                        calcualtePositions2DFrom3D(),
    //                    mPositionData.getPositions(),
                        mData->getTopology()->getOuterTopology().getFacesIndices(),
                        mOuterFaceNormals.getVectors());
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

std::shared_ptr<Polygon2DAccessor> Polygon3D::createAccessor()
{
    class OuterPolygon2DAccessor : public Polygon2DAccessor
    {
    public:
        OuterPolygon2DAccessor(const std::shared_ptr<Polygon3D>& _poly3)
            : poly3(_poly3)
        {

        }

        virtual ~OuterPolygon2DAccessor() override
        {

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
        std::shared_ptr<Polygon3D> poly3;
    };

    return std::make_shared<OuterPolygon2DAccessor>(
                std::dynamic_pointer_cast<Polygon3D>(shared_from_this()));
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

    mOuterVertexNormals.setTransform(Affine3d(transform.rotation()));
    mOuterFaceNormals.setTransform(Affine3d(transform.rotation()));
}

void Polygon3D::fixOuterTriangleIndexOrder(bool printInfo)
{
    std::shared_ptr<Polygon3DTopology> topology = mData->getTopology();

    synchronizeTriangleIndexOrder();

    int fixedFacesCounter = 0;

    for (TopologyFace& outerFace2D : topology->getOuterTopology().getFaces())
    {
        ID face3DId = topology->to3DFaceIndex(outerFace2D.getID());
        TopologyFace& face3D = topology->getFace(face3DId);

        if (face3D.getCellIds().empty())
            continue;

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

        Vector r1 = mPositionData.getPosition(f[1]) -
                mPositionData.getPosition(f[0]);
        Vector r2 = mPositionData.getPosition(f[2]) -
                mPositionData.getPosition(f[0]);
        Vector r3 = mPositionData.getPosition(innerVertexId) -
                mPositionData.getPosition(f[0]);

        // Reverts the face indices so that its normal:
        // (x2 - x0).cross(x1 - x0).normalized()
        // points in the other direction.
        auto revertFace = [](Face& f)
        {
            unsigned int temp = f[1];
            f[1] = f[2];
            f[2] = temp;
        };

        if (r1.cross(r2).normalized().dot(r3) > 0)
        {
            revertFace(f);
            revertFace(outerFace2D.getVertexIds());
            revertFace(topology->getOuterFacesIndices3D()[outerFace2D.getID()]);

            ++fixedFacesCounter;
        }
    }

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
            topology->getOuterFacesIndices3D()[outerFace2D.getID()][i] = index3d;
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
