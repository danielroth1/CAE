#include "Polygon3D.h"

#include "GeometricDataUtils.h"
#include "Polygon3DDataBS.h"
#include "Polygon3DDataWS.h"
#include "Polygon3DTopology.h"

#include <scene/data/GeometricDataVisitor.h>
#include <scene/model/ModelUtils.h>
#include <set>

Polygon3D::Polygon3D(
        const Vectors& positionsWS,
        const Polygon3DTopology& topology)
    : Polygon(positionsWS)
{
    // World space construtor
    Polygon::initWorldSpace(positionsWS);
    mData = std::make_shared<Polygon3DDataWS>(topology);

    mOuterVertexNormals.initializeFromWorldSpace(
                GeometricDataUtils::calculateNormals(
                    positionsWS,
                    topology.getOuterFacesIndices3D()));

    // initialize mOuterFaceNormals
    Vectors faceNormals;
//    ModelUtils::calculateFaceNormals<double>(
//                mPositionData.getPositions(), faces, faceNormals);
    ModelUtils::calculateFaceNormals<double>(
                mPositionData.getPositions(),
                topology.getOuterFacesIndices3D(),
                faceNormals);

    mOuterFaceNormals.initializeFromWorldSpace(faceNormals);
}

Polygon3D::Polygon3D(
        const Vectors& positionsWS,
        const Vectors& vertexNormalsWS,
        const Polygon3DTopology& topology)
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
                topology.getOuterFacesIndices3D(),
                faceNormals);

    mOuterFaceNormals.initializeFromWorldSpace(faceNormals);
}

Polygon3D::Polygon3D(
        Vectors* positionsBS,
        const Affine3d& transform,
        const Polygon3DTopology& topology)
    : Polygon(positionsBS, transform)
{
    Vectors faceNormals = GeometricDataUtils::calculateNormals(
                *positionsBS, topology.getOuterFacesIndices3D());
    std::shared_ptr<Polygon3DDataBS> dataBS =
            std::make_shared<Polygon3DDataBS>(
                topology,
                mPositionData.getPositionsBS(),
                faceNormals,
                ModelUtils::calculateFaceNormals<double>(
                    mPositionData.getPositions(),
                    topology.getOuterFacesIndices3D(),
                    faceNormals));

    mData = dataBS;
    Polygon::initBodySpace(&dataBS->getPositionsBS(), transform);
    mOuterVertexNormals.initializeFromBodySpace(&dataBS->getOuterVertexNormalsBS(),
                                                Eigen::Affine3d(transform.linear()));
    mOuterFaceNormals.initializeFromBodySpace(&dataBS->getOuterFaceNormalsBS(),
                                              Eigen::Affine3d(transform.linear()));
}

Polygon3D::Polygon3D(
        Vectors* positionsBS,
        const Affine3d& transform,
        const Vectors& vertexNormalsBS,
        const Polygon3DTopology& topology)
    : Polygon (positionsBS, transform)
{
    Vectors faceNormals = GeometricDataUtils::calculateNormals(
                *positionsBS, topology.getOuterFacesIndices3D());
    std::shared_ptr<Polygon3DDataBS> dataBS = std::make_shared<Polygon3DDataBS>(
                topology,
                mPositionData.getPositionsBS(),
                vertexNormalsBS,
                ModelUtils::calculateFaceNormals<double>(
                    mPositionData.getPositions(),
                    topology.getOuterFacesIndices3D(),
                    faceNormals));

    mData = dataBS;
    Polygon::initBodySpace(&dataBS->getPositionsBS(), transform);
    mOuterVertexNormals.initializeFromBodySpace(&dataBS->getOuterVertexNormalsBS(),
                                                Eigen::Affine3d(transform.linear()));
    mOuterFaceNormals.initializeFromBodySpace(&dataBS->getOuterFaceNormalsBS(),
                                              Eigen::Affine3d(transform.linear()));
}

Polygon3D::~Polygon3D()
{

}

Vectors Polygon3D::calcualtePositions2DFrom3D() const
{
    std::vector<unsigned int>& outerVertexIds = mData->getTopology().getOuterVertexIds();
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
    return mData->getTopology();
}

const Polygon3DTopology& Polygon3D::getTopology3D() const
{
    return mData->getTopology();
}

std::vector<unsigned int>& Polygon3D::getOuterPositionIds()
{
    return mData->getTopology().getOuterVertexIds();
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

void Polygon3D::update()
{
    Polygon::update();

    if (mOuterVertexNormals.getType() == BSWSVectors::Type::BODY_SPACE)
    {
        mOuterVertexNormals.update();
        mOuterFaceNormals.update();
    }
    else
    {
        Vectors normalsTemp;
        ModelUtils::calculateNormals<double>(
                    calcualtePositions2DFrom3D(),
//                    mPositionData.getPositions(),
                    mData->getTopology().getOuterTopology().getFacesIndices(),
                    normalsTemp);

        mOuterVertexNormals.getVectors() = normalsTemp;

        ModelUtils::calculateFaceNormals<double>(
                    calcualtePositions2DFrom3D(),
//                    mPositionData.getPositions(),
                    mData->getTopology().getOuterTopology().getFacesIndices(),
                    normalsTemp);

        mOuterFaceNormals.getVectors() = normalsTemp;
    }
}

bool Polygon3D::isInside(const TopologyFeature& feature, Vector point)
{
    return Polygon::isInside(
                feature,
                point,
                mData->getTopology().getOuterTopology(),
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
                mData->getTopology().getOuterTopology(), mOuterFaceNormals);
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
    return mData->getTopology();
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
            poly3->setPosition(poly3->mData->getTopology().to2DIndex(index), position);
        }

        virtual Eigen::Vector& getPosition(std::size_t index) override
        {
            return poly3->getPosition(poly3->mData->getTopology().to2DIndex(index));
        }

        virtual Polygon2DTopology& getTopology2D() override
        {
            return poly3->mData->getTopology().getOuterTopology();
        }

        virtual const Polygon2DTopology& getTopology2D() const override
        {
            return poly3->mData->getTopology().getOuterTopology();
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

bool Polygon3D::isInside(ID faceId, const Vector& point,
                         PolygonTopology& topology, BSWSVectors& faceNormals)
{
    ID vertexId = topology.getFace(faceId).getVertexIds()[0];
    return faceNormals.getVector(faceId)
            .dot(point - mPositionData.getPosition(mData->getTopology().getOuterVertexIds()[vertexId])) < 0;
}
