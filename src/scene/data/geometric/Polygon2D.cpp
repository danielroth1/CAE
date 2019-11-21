#include "Polygon2D.h"

#include "GeometricDataUtils.h"
#include "Polygon2DDataBS.h"
#include "Polygon2DDataWS.h"
#include "Polygon2DTopology.h"
#include "TopologyFeature.h"
#include "TopologyVertex.h"

#include <iostream>

#include <scene/data/GeometricDataVisitor.h>

#include <scene/model/ModelUtils.h>

using namespace Eigen;

Polygon2D::Polygon2D(
        const Vectors& positionsWS,
        const Faces& faces)
    : Polygon(positionsWS)
{
    // World space construtor
    Polygon::initWorldSpace(positionsWS);
    mData = std::make_shared<Polygon2DDataWS>(
                Polygon2DTopology(faces, positionsWS.size()));

    Vectors vertexNormals = GeometricDataUtils::calculateNormals(
                mPositionData.getPositions(), faces);
    mVertexNormals.initializeFromWorldSpace(vertexNormals);

    Vectors faceNormals;
    ModelUtils::calculateFaceNormals<double>(
                mPositionData.getPositions(), faces, faceNormals);
    mFaceNormals.initializeFromWorldSpace(faceNormals);

//    fixTopology();

    mAccessor2D = createAccessor();
}

Polygon2D::Polygon2D(
        const Vectors& positionsWS,
        const Vectors& vertexNormalsWS,
        const Faces& faces)
    : Polygon(positionsWS)
{
    // World space constructor
    Polygon::initWorldSpace(positionsWS);
    mData = std::make_shared<Polygon2DDataWS>(
                Polygon2DTopology(faces, positionsWS.size()));
    mVertexNormals.initializeFromWorldSpace(vertexNormalsWS);

    Vectors faceNormals;
    ModelUtils::calculateFaceNormals<double>(
                mPositionData.getPositions(), faces, faceNormals);
    mFaceNormals.initializeFromWorldSpace(faceNormals);

//    fixTopology();

    mAccessor2D = createAccessor();
}

Polygon2D::Polygon2D(
        const Vectors& positionsBS,
        const Affine3d& transform,
        const Faces& faces)
    : Polygon()
{
    // Body space construtor

    Vectors vertexNormals = GeometricDataUtils::calculateNormals(
                mPositionData.getPositions(), faces);

    Vectors faceNormals;
    ModelUtils::calculateFaceNormals<double>(
                mPositionData.getPositions(), faces, faceNormals);

    std::shared_ptr<Polygon2DDataBS> dataBS =
            std::make_shared<Polygon2DDataBS>(
                Polygon2DTopology(faces, positionsBS.size()),
                positionsBS,
                vertexNormals,
                faceNormals);

    mData = dataBS;
    Polygon::initBodySpace(&dataBS->getPositionsBS(), transform);
    mVertexNormals.initializeFromBodySpace(&dataBS->getVertexNormalsBS(),
                                           Eigen::Affine3d(transform.linear()));
    mFaceNormals.initializeFromBodySpace(&dataBS->getFaceNormalsBS(),
                                           Eigen::Affine3d(transform.linear()));

//    fixTopology();

    mAccessor2D = createAccessor();
}

Polygon2D::Polygon2D(
        const Vectors& positionsBS,
        const Eigen::Affine3d& transform,
        const Vectors& vertexNormalsBS,
        const Faces& faces)
    : Polygon()
    , mFaceNormals(faces.size())
{
    Vectors faceNormals;
    ModelUtils::calculateFaceNormals<double>(
                mPositionData.getPositions(), faces, faceNormals);

    // Body space constructor
    std::shared_ptr<Polygon2DDataBS> dataBS =
            std::make_shared<Polygon2DDataBS>(
                Polygon2DTopology(faces, positionsBS.size()),
                positionsBS,
                vertexNormalsBS,
                faceNormals);

    mData = dataBS;
    Polygon::initBodySpace(&dataBS->getPositionsBS(), transform);
    mVertexNormals.initializeFromBodySpace(&dataBS->getVertexNormalsBS(),
                                           Eigen::Affine3d(transform.linear()));
    mFaceNormals.initializeFromBodySpace(&dataBS->getFaceNormalsBS(),
                                         Eigen::Affine3d(transform.linear()));

//    fixTopology();

    mAccessor2D = createAccessor();
}

Polygon2DTopology& Polygon2D::getTopology2D()
{
    return mData->getTopology();
}

const Polygon2DTopology& Polygon2D::getTopology2D() const
{
    return mData->getTopology();
}

std::shared_ptr<Polygon2DData> Polygon2D::getData2D()
{
    return mData;
}

Vectors& Polygon2D::getVertexNormals()
{
    return mVertexNormals.getVectors();
}

Vectors& Polygon2D::getFaceNormals()
{
    return mFaceNormals.getVectors();
}

void Polygon2D::accept(GeometricDataVisitor& visitor)
{
    visitor.visit(*this);
}

void Polygon2D::updateBoundingBox()
{
    mPositionData.update();
    GeometricDataUtils::updateBoundingBox(
                mPositionData.getPositions(),
                mBoundingBox);
}

void Polygon2D::update(bool updateFaceNormals, bool updateVertexNormals)
{
    Polygon::update(updateFaceNormals, updateVertexNormals);

    if (mVertexNormals.getType() == BSWSVectors::Type::BODY_SPACE)
    {
        if (updateVertexNormals)
            mVertexNormals.update();

        if (updateFaceNormals)
            mFaceNormals.update();
    }
    else
    {
        if (updateVertexNormals)
        {
            ModelUtils::calculateNormals<double>(
                        mPositionData.getPositions(),
                        mData->getTopology().getFacesIndices(),
                        mVertexNormals.getVectors());
        }

        if (updateFaceNormals)
        {
            ModelUtils::calculateFaceNormals<double>(
                        mPositionData.getPositions(),
                        mData->getTopology().getFacesIndices(),
                        mFaceNormals.getVectors());
        }

    }
}

void Polygon2D::fixTopology()
{
    std::set<ID> notReferenced;

    std::vector<ID> notReferencedByEdges =
            mData->getTopology().retrieveNotReferencedByEdges();
    notReferenced.insert(notReferencedByEdges.begin(),
                         notReferencedByEdges.end());

    std::vector<ID> notReferencedByFaces =
            mData->getTopology().retrieveNotReferencedByFaces();
    notReferenced.insert(notReferencedByFaces.begin(),
                         notReferencedByFaces.end());

    if (notReferencedByEdges.size() > 0)
        std::cout << "remove " << notReferencedByEdges.size() << " vertices that "
                  << "are not referenced by edges.\n";

    if (notReferencedByFaces.size() > 0)
        std::cout << "remove " << notReferencedByFaces.size() << " vertices that "
                  << "are not referenced by faces.\n";

    std::vector<ID> notReferencedVector;
    notReferencedVector.insert(notReferencedVector.end(),
                         notReferenced.begin(),
                         notReferenced.end());

    // TODO: should not be necessary becaues elements are already sorted in
    // set and vector is sorted anyways in remove method.
    std::sort(notReferencedVector.begin(), notReferencedVector.end());

    if (notReferencedVector.size() > 0)
        std::cout << "remove a total of " << notReferencedVector.size()
                  << " vertices\n";

    removeVertices(notReferencedVector);
}

void Polygon2D::removeVertex(ID index)
{
    Polygon::removeVertex(index);
    mData->removeVector(index);
    mVertexNormals.removeVector(index);
    mFaceNormals.removeVector(index);
}

void Polygon2D::removeVertices(std::vector<ID>& indices)
{
    if (indices.empty())
        return;

    Polygon::removeVertices(indices);
    mData->removeVectors(indices);
    mFaceNormals.removeVectors(indices.begin(), indices.end());
    mVertexNormals.removeVectors(indices.begin(), indices.end());
}

bool Polygon2D::isInside(const TopologyFeature& feature, Vector point)
{
    return Polygon::isInside(feature, point, mData->getTopology(), mFaceNormals);
}

bool Polygon2D::isInside(
        const TopologyFeature& feature,
        Vector source,
        double distance,
        Vector target)
{
    return Polygon::isInside(feature, source, distance, target,
                             mData->getTopology(), mFaceNormals);
}

Polygon::DimensionType Polygon2D::getDimensionType() const
{
    return DimensionType::TWO_D;
}

std::shared_ptr<PolygonData> Polygon2D::getData()
{
    return mData;
}

PolygonTopology& Polygon2D::getTopology()
{
    return mData->getTopology();
}

const std::shared_ptr<Polygon2DAccessor>& Polygon2D::getAccessor2D() const
{
    return mAccessor2D;
}

std::shared_ptr<Polygon2DAccessor> Polygon2D::createAccessor()
{
    class Polygon2DAccessorImpl : public Polygon2DAccessor
    {
    public:
        Polygon2DAccessorImpl(const std::shared_ptr<Polygon2D>& _poly2)
            : poly2(_poly2)
        {

        }

        virtual ~Polygon2DAccessorImpl() override
        {

        }

        virtual bool isInside(
                const TopologyFeature& feature, const Eigen::Vector& source,
                double distance, const Eigen::Vector& target) override
        {
            return poly2->isInside(feature, source, distance, target);
        }

        virtual std::shared_ptr<Polygon> getPolygon() const override
        {
            return poly2;
        }

        virtual size_t getSize() override
        {
            return poly2->getSize();
        }

        virtual void setPosition(size_t index, const Eigen::Vector& position) override
        {
            poly2->setPosition(index, position);
        }

        virtual Eigen::Vector& getPosition(std::size_t index) override
        {
            return poly2->getPosition(index);
        }

        virtual Polygon2DTopology& getTopology2D() override
        {
            return poly2->getTopology2D();
        }

        virtual const Polygon2DTopology& getTopology2D() const override
        {
            return poly2->getTopology2D();
        }

        virtual Vectors& getVertexNormals() override
        {
            return poly2->getVertexNormals();
        }

        virtual Vectors& getFaceNormals() override
        {
            return poly2->getFaceNormals();
        }

    private:
        std::shared_ptr<Polygon2D> poly2;
    };

    return std::make_shared<Polygon2DAccessorImpl>(
                std::dynamic_pointer_cast<Polygon2D>(shared_from_this()));
}

//void Polygon2D::changeRepresentationToBS(
//        Vectors* vectorsBS,
//        const Affine3d& transform)
//{
//    Polygon::changeRepresentationToBS(vectorsBS, transform);
//    mVertexNormals.changeRepresentationToBS();
//    mFaceNormals.changeRepresentationToBS();
//}

void Polygon2D::changeRepresentationToBS(const Eigen::Vector& center)
{
    std::shared_ptr<Polygon2DDataWS> dataWS =
            std::dynamic_pointer_cast<Polygon2DDataWS>(mData);
    if (dataWS)
    {
        std::shared_ptr<Polygon2DDataBS> dataBS =
                std::make_shared<Polygon2DDataBS>(mData->getTopology(),
                                                  mPositionData.getPositions(),
                                                  mVertexNormals.getVectors(),
                                                  mFaceNormals.getVectors());
        mData = dataBS;

        mPositionData.changeRepresentationToBS(
                    &dataBS->getPositionsBS(), Eigen::Affine3d::Identity());
        mPositionData.moveCenterTo(center);

        mVertexNormals.changeRepresentationToBS(
                    &dataBS->getVertexNormalsBS(), Eigen::Affine3d::Identity());
        mFaceNormals.changeRepresentationToBS(
                    &dataBS->getFaceNormalsBS(), Eigen::Affine3d::Identity());
    }
}

void Polygon2D::changeRepresentationToWS()
{
    std::shared_ptr<Polygon2DDataBS> dataBS =
            std::dynamic_pointer_cast<Polygon2DDataBS>(mData);
    if (dataBS)
    {
        std::shared_ptr<Polygon2DDataWS> dataWS =
                std::make_shared<Polygon2DDataWS>(mData->getTopology());
        mData = dataWS;

        mPositionData.changeRepresentationToWS();
        mVertexNormals.changeRepresentationToWS();
        mFaceNormals.changeRepresentationToWS();
    }
}

void Polygon2D::setTransform(const Affine3d& transform)
{
    Polygon::setTransform(transform);

    mVertexNormals.setTransform(Affine3d(transform.rotation()));
    mFaceNormals.setTransform(Affine3d(transform.rotation()));
}

