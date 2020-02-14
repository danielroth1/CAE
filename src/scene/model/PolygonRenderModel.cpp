#include "ModelUtils.h"
#include "PolygonIndexMapping.h"
#include "PolygonRenderModel.h"
#include "RenderModelVisitor.h"

#include <iostream>

#include <scene/data/GeometricDataVisitor.h>
#include <scene/data/geometric/GeometricDataListener.h>
#include <scene/data/geometric/GeometricPoint.h>
#include <scene/data/geometric/Polygon.h>
#include <scene/data/geometric/Polygon2D.h>
#include <scene/data/geometric/Polygon2DData.h>
#include <scene/data/geometric/Polygon3D.h>
#include <scene/data/geometric/Polygon3DTopology.h>
#include <scene/data/geometric/PolygonData.h>
#include <scene/data/geometric/TopologyFace.h>

#include <times/timing.h>

#include <rendering/object/RenderLines.h>
#include <rendering/object/RenderPoints.h>
#include <rendering/object/RenderPolygons.h>
#include <rendering/object/RenderPolygonsConstantData.h>
#include <rendering/object/RenderPolygonsConstantDataBS.h>
#include <rendering/object/RenderPolygonsConstantDataWS.h>
#include <rendering/object/RenderPolygonsData.h>
#include <rendering/object/RenderPolygonsDataBS.h>
#include <rendering/object/RenderPolygonsDataWS.h>

#include <rendering/Appearance.h>
#include <rendering/Appearances.h>
#include <rendering/Renderer.h>

#include <RenderModelManager.h>

#include <data_structures/VectorIndexMapping.h>

PolygonRenderModel::PolygonRenderModel(
        RenderModelManager* renderModelManager,
        std::shared_ptr<Polygon> polygon,
        bool renderOnlyOuterFaces,
        bool renderVertexNormals,
        bool renderFaceNormals)
    : mRenderModelManager(renderModelManager)
    , mPolygon(polygon)
    , mRenderOnlyOuterFaces(renderOnlyOuterFaces)
    , mRenderVertexNormals(renderVertexNormals)
    , mRenderFaceNormals(renderFaceNormals)
{
    mRequiresUpdate = false;
    mCurrentlyRenderedCenter = Eigen::Vector::Zero();

    mRenderPolygons = nullptr;

    mCurrentlyRenderedTransform.setIdentity();

    // Resets mPositionsBufferedData and mNormalsBufferedData and resizes them.
    // Initializes mFacesBufferedData from mPolygon.
    reset();
    updatePositions();

    // Add GeometricDataListener to given GeometricData that checks
    // if the data requires updates. Updates are only performed
    // if it does require them.
    class PolygonListener : public GeometricDataListener
    {
    public:
        PolygonListener(PolygonRenderModel& _model)
            : model(_model)
        {
        }

        virtual void notifyGeometricDataChanged()
        {
            model.mRequiresUpdate = true;
        }

        PolygonRenderModel& model;
    };

    mGeometricListener = std::make_shared<PolygonListener>(*this);
    polygon->addGeometricDataListener(mGeometricListener);
}

PolygonRenderModel::~PolygonRenderModel()
{
    // remove listener that was created in the constructor
    mPolygon->removeGeometricDataListener(mGeometricListener.get());

    mRenderModelManager->removePolygonData(mPolygon->getData());

    if (mRenderPolygons && mRenderPolygonsData)
        mRenderPolygons->removeRenderPolygonsData(mRenderPolygonsData);
}

void PolygonRenderModel::setTextureCoordinates(
        const std::vector<Vector2f>& textureCoordinates)
{
    mRenderPolygonsData->setTextureCoordinates(textureCoordinates);
}

void PolygonRenderModel::setRenderedAppearances(
        const std::shared_ptr<Appearances>& appearances)
{
    mRenderPolygonsData->setAppearances(appearances);
}

std::shared_ptr<Appearances> PolygonRenderModel::getRenderedAppearances()
{
    return mRenderPolygonsData->getAppearances();
}

void PolygonRenderModel::setPolygonIndexMapping(
        const std::shared_ptr<PolygonIndexMapping>& poylgonIndexMapping)
{
    mPolygonIndexMapping = poylgonIndexMapping;
}

bool PolygonRenderModel::isTexturingEnabled() const
{
    return mRenderPolygonsData->isTexturingEnabled();
}

void PolygonRenderModel::setTexturingEnabled(bool texturingEnabled)
{
    mRenderPolygonsData->setTexturingEnabled(texturingEnabled);
}

bool PolygonRenderModel::isRenderOnlyOuterFaces() const
{
    return mRenderOnlyOuterFaces;
}

void PolygonRenderModel::setRenderOnlyOuterFaces(bool renderOnlyOuterFaces)
{
    if (mRenderOnlyOuterFaces != renderOnlyOuterFaces)
    {
        mRenderOnlyOuterFaces = renderOnlyOuterFaces;
        reset();
    }
}

bool PolygonRenderModel::isRenderVertexNormals() const
{
    return mRenderVertexNormals;
}

void PolygonRenderModel::setRenderVertexNormals(bool renderVertexNormals)
{
    mRenderVertexNormals = renderVertexNormals;

    INVOKE_METHOD(PolygonRenderModel, shared_from_this(),
                  revalidatePointLineRendering, mRenderer->getDomain()); // my synchronized implementation
}

bool PolygonRenderModel::isRenderFaceNormals() const
{
    return mRenderFaceNormals;
}

void PolygonRenderModel::setRenderFaceNormals(bool renderFaceNormals)
{
    mRenderFaceNormals = renderFaceNormals;
    INVOKE_METHOD(PolygonRenderModel, shared_from_this(),
                  revalidatePointLineRendering, mRenderer->getDomain());
}

void PolygonRenderModel::reset()
{
    // Init render object
//    mRenderPoly2 = std::make_shared<RenderPolygon2D>();

    // initialize mRenderPolygonsData and mRenderPolygons

    // RenderPolygonsData is Polygon specific and, therefore, must be created
    // for body and world space configuration.
    // RenderPolygons can be shared by multiple Polygons but only if the Polygons
    // share certain data.

    bool replacedRenderPolygons = false;
    mCurrentType = mPolygon->getPositionType();

    std::shared_ptr<PolygonData> data = mPolygon->getData();

    std::shared_ptr<RenderPolygons> renderPolygons =
            mRenderModelManager->getRenderPolygons(data);

    // Create RenderPolygonsData that is specific to this
    // polygon.

    if (mRenderPolygonsData)
    {
        mRenderPolygons->removeRenderPolygonsData(mRenderPolygonsData);

        // World space uses its own vertex positions and ignored transformation
        // matrix.
        if (mRenderPolygons->getType() == BSWSVectors::Type::WORLD_SPACE)
        {
            std::static_pointer_cast<RenderPolygonsDataBS>(mRenderPolygonsData)
                    ->getTransform()->setIdentity();
        }
    }

    // set mRenderPolygons and create RenderPolygonsConstantData
    // if necessary
    if (renderPolygons &&
        renderPolygons->getType() == mPolygon->getPositionType())
    {
        mRenderPolygons = renderPolygons;
    }
    else
    {
//            if (renderPolygons &&
//                renderPolygons->getType() != mPolygon->getPositionType())
//            {
//                // mRenderPolygons is there but of different positions type
//                mRenderPolygons->removeRenderPolygonsData(mRenderPolygonsData);
//            }
        replacedRenderPolygons = true;

        switch(mPolygon->getPositionType())
        {
        case BSWSVectors::Type::BODY_SPACE:
        {
            std::shared_ptr<RenderPolygonsConstantDataBS> constantData =
                    std::make_shared<RenderPolygonsConstantDataBS>();
            mRenderPolygons = std::make_shared<RenderPolygons>(constantData);
            break;
        }
        case BSWSVectors::Type::WORLD_SPACE:
        {
            std::shared_ptr<RenderPolygonsConstantDataWS> constantData =
                    std::make_shared<RenderPolygonsConstantDataWS>();
            mRenderPolygons = std::make_shared<RenderPolygons>(constantData);
            break;
        }
        }
    }

    mRenderPolygons->setVisible(true);
    mRenderModelManager->addPolygonData(
                mPolygon->getData(), mRenderPolygons);

    switch(mPolygon->getPositionType())
    {
    case BSWSVectors::Type::BODY_SPACE:
    {
        if (mRenderPolygonsData)
            mRenderPolygonsData = std::make_shared<RenderPolygonsDataBS>(
                        *mRenderPolygonsData.get());
        else
            mRenderPolygonsData = std::make_shared<RenderPolygonsDataBS>();
        break;
    }
    case BSWSVectors::Type::WORLD_SPACE:
    {
        if (mRenderPolygonsData)
            mRenderPolygonsData = std::make_shared<RenderPolygonsDataWS>(
                        *mRenderPolygonsData.get());
        else
            mRenderPolygonsData = std::make_shared<RenderPolygonsDataWS>();
        break;
    }
    }

    if (!mRenderPolygonsData->getAppearances())
    {
        setAppearances(std::make_shared<Appearances>(
                           Appearance::createDefaultAppearance()));
    }

    initializeBufferedData();

    // reset positions
    size_t size;

    if (mPolygonIndexMapping)
    {
        size = mPolygonIndexMapping->getVectorIndexMapping()->getExtendedSize();
    }
    else
    {
        switch(mPolygon->getDimensionType())
        {
        case Polygon::DimensionType::TWO_D:
        {
            Polygon2D* p2 = static_cast<Polygon2D*>(mPolygon.get());
            size = p2->getPositions().size();

            break;
        }
        case Polygon::DimensionType::THREE_D:
        {
            Polygon3D* p3 = static_cast<Polygon3D*>(mPolygon.get());

            if (mRenderOnlyOuterFaces)
            {
                size = p3->getOuterPositionIds().size();
            }
            else
            {
                size = p3->getPositions().size();
            }

            break;
        }
        }
    }

    mPositionsBufferedData->getData().lock()->resize(size);
    mNormalsBufferedData->getData().lock()->resize(size);

    std::vector<TopologyFace>* faces = retrieveRelevantFaces();
    {
        auto facesLock = mFacesBufferedData->getData().lock();
        facesLock->resize(faces->size());

        if (mPolygonIndexMapping)
        {
            for (size_t i = 0; i < faces->size(); ++i)
            {
                facesLock->at(i) = mPolygonIndexMapping->getFaces()[i];
            }
        }
        else
        {
            for (size_t i = 0; i < faces->size(); ++i)
            {
                facesLock->at(i) = (*faces)[i].getVertexIds();
            }
        }

    }
    mFacesBufferedData->setDataChanged(true);

    // render normals
    revalidatePointLineRendering();

    updatePositions();

//    if (replacedRenderPolygons)
        mAddedToRenderer = false;
}

void PolygonRenderModel::update()
{
    // update the transformation matrix if it changed by a certain degree

    if (!mRequiresUpdate)
        return;

    if (mPolygon->getPositionType() != mCurrentType)
    {
        reset();
    }

    mRequiresUpdate = false;

    switch(mPolygon->getPositionType())
    {
    case BSWSVectors::BODY_SPACE:
    {
        // All vertices need to be udpated when the center changed. The center
        // changes only once at the start. This is not the position.
        Vector currentCenter = mPolygon->getCenter();
        if (!mCurrentlyRenderedCenter.isApprox(currentCenter, 1e-3))
        {
            mCurrentlyRenderedCenter = currentCenter;
            updatePositions();
        }

        // only need to udpate the transformation matrix. No need
        // for moving every vertex to the GPU (which is equal to a
        // revalidation in this case).
        if (!mCurrentlyRenderedTransform.isApprox(mPolygon->getTransform(), 1e-3))
        {
            updateTransform();
        }

        break;
    }
    case BSWSVectors::WORLD_SPACE:
        updatePositions();
        break;
    }

    if (mRenderLinesNormals)
        updateNormalLines();
}

void PolygonRenderModel::revalidate()
{
    reset();
    mRequiresUpdate = true;
    update();
}

void PolygonRenderModel::accept(RenderModelVisitor& v)
{
    v.visit(*this);
}

void PolygonRenderModel::addToRenderer(Renderer* renderer)
{
    mRenderer = renderer;

    mRenderPolygons->setDomain(renderer->getDomain());
    mRenderPolygons->addRenderPolygonsData(mRenderPolygonsData);
    renderer->addRenderObject(mRenderPolygons);
//    renderer->addRenderObject(mRenderPolygonsData);

    if (mRenderLinesNormals)
        renderer->addRenderObject(mRenderLinesNormals);

    if (mRenderPoints)
        renderer->addRenderObject(mRenderPoints);
}

void PolygonRenderModel::removeFromRenderer(Renderer* renderer)
{
    mRenderPolygons->removeRenderPolygonsData(mRenderPolygonsData);
    if (mRenderPolygons->getData().size() == 0)
        renderer->removeRenderObject(mRenderPolygons);
//    renderer->removeRenderObject(mRenderPolygonsData);

    if (mRenderLinesNormals)
        renderer->removeRenderObject(mRenderLinesNormals);

    if (mRenderPoints)
        renderer->removeRenderObject(mRenderPoints);
}

void PolygonRenderModel::setVisible(bool visible)
{
    mRenderPolygonsData->setVisible(visible);
    if (mRenderVertexNormals)
    {
        mRenderLinesNormals->setVisible(visible);
        mRenderPoints->setVisible(visible);
    }
    RenderModel::setVisible(visible);
}

void PolygonRenderModel::setWireframeEnabled(bool wireframeEnabled)
{
    mRenderPolygonsData->setWireframeEnabled(wireframeEnabled);
    RenderModel::setWireframeEnabled(wireframeEnabled);
}

std::vector<TopologyFace>* PolygonRenderModel::retrieveRelevantFaces()
{
    class GetFacesVisitor : public GeometricDataVisitor
    {
    public:
        GetFacesVisitor(PolygonRenderModel& _rm)
            : rm(_rm)
        {

        }

        virtual void visit(Polygon2D& polygon2D)
        {
            faces = &polygon2D.getTopology().getFaces();
        }

        virtual void visit(Polygon3D& polygon3D)
        {
            if (rm.mRenderOnlyOuterFaces)
                faces = &polygon3D.getTopology3D().getOuterFaces();
            else
                faces = &polygon3D.getTopology3D().getFaces();
        }

        virtual void visit(GeometricPoint& /*point*/)
        {

        }

        PolygonRenderModel& rm;
        std::vector<TopologyFace>* faces;
    } visitor(*this);

    mPolygon->accept(visitor);
    return visitor.faces;
}

void PolygonRenderModel::initializeBufferedData()
{
    mTexturesCoordinatesBufferedData =
            &mRenderPolygonsData->getTexturesCoordinatesBufferedData();

    switch(mRenderPolygons->getType())
    {
    case BSWSVectors::BODY_SPACE:
    {
        std::shared_ptr<RenderPolygonsDataBS> dataBS =
                std::static_pointer_cast<RenderPolygonsDataBS>(mRenderPolygonsData);
        std::shared_ptr<RenderPolygonsConstantDataBS> constantDataBS =
                std::static_pointer_cast<RenderPolygonsConstantDataBS>(mRenderPolygons->getConstantData());

        // set a flag so that renderer knows that refresh of buffer is
        // necessary or better, call some operation that triggers that
        // refresh once.
        mPositionsBufferedData = &constantDataBS->getPositionsBuffer();
        mNormalsBufferedData = &constantDataBS->getNormalsBuffer();
        mFacesBufferedData = &constantDataBS->getFacesBuffer();

        break;
    }
    case BSWSVectors::WORLD_SPACE:
    {
        std::shared_ptr<RenderPolygonsDataWS> dataWS =
                std::static_pointer_cast<RenderPolygonsDataWS>(mRenderPolygonsData);
        std::shared_ptr<RenderPolygonsConstantDataWS> constantDataWS =
                std::static_pointer_cast<RenderPolygonsConstantDataWS>(mRenderPolygons->getConstantData());

        mPositionsBufferedData = &dataWS->getPositionsBuffer();
        mNormalsBufferedData = &dataWS->getNormalsBuffer();
        mFacesBufferedData = &constantDataWS->getFacesBuffer();

        break;
    }
    }
}

void PolygonRenderModel::revalidatePointLineRendering()
{
    // set invisible if neccessary
    if (mRenderVertexNormals || mRenderFaceNormals)
    {
        // Create and add (if not already done) renderLinesNormals and renderPoints
        // to renderer.

        if (!mRenderLinesNormals)
        {
            mRenderLinesNormals = std::make_shared<RenderLines>();
            // yellow
            mRenderLinesNormals->setRenderMaterial(
                        RenderMaterial::createFromColor({1.0f, 1.0f, 0.0f, 1.0f}));
            mRenderer->addRenderObject(mRenderLinesNormals);
        }

        if (!mRenderPoints)
        {
            mRenderPoints = std::make_shared<RenderPoints>();
            // orange
            mRenderPoints->setRenderMaterial(
                        RenderMaterial::createFromColor({1.0f, 1.0f, 0.5f, 1.0f}));
            mRenderer->addRenderObject(mRenderPoints);
        }
    }
    else if (!mRenderVertexNormals && !mRenderFaceNormals)
    {
        // renderLineNormals and renderPoints aren't needed anymore, so remove
        // them from renderer
        if (mRenderLinesNormals)
        {
            mRenderer->removeRenderObject(mRenderLinesNormals);
            mRenderLinesNormals = nullptr;
        }

        if (mRenderPoints)
        {
            mRenderer->removeRenderObject(mRenderPoints);
            mRenderPoints = nullptr;
        }

    }
}

void PolygonRenderModel::updatePositions()
{

    Vectors& positions
            = mPolygon->getPositionType() == BSWSVectors::BODY_SPACE ?
                mPolygon->getPositionsBS() :
                mPolygon->getPositions(); // for 3d case, this are all 3d positions, but often only 2d positions are required

    {
        START_TIMING_MODELLING("PolygonRenderModel::updatePositions::positions");
        auto positionsLock = mPositionsBufferedData->getData().lock();

        if (mPolygonIndexMapping)
        {
            std::shared_ptr<VectorIndexMapping> vim =
                    mPolygonIndexMapping->getVectorIndexMapping();

            for (size_t i = 0; i < vim->getExtendedSize(); ++i)
            {
                size_t index = vim->getOriginalIndex(i);
                if (mRenderOnlyOuterFaces && mPolygon->getDimensionType() == Polygon::DimensionType::THREE_D)
                {
                    positionsLock->at(i) = positions[
                            static_cast<Polygon3D*>(
                                mPolygon.get())->getOuterPositionIds()[index]].cast<float>();
                }
                else
                {
                    positionsLock->at(i) = positions[index].cast<float>();
                }
            }
        }
        else
        {
            for (size_t i = 0; i < positionsLock->size(); ++i)
            {
                if (mRenderOnlyOuterFaces &&
                    mPolygon->getDimensionType() == Polygon::DimensionType::THREE_D)
                {
                    positionsLock->at(i) = positions[
                            static_cast<Polygon3D*>(mPolygon.get())->
                            getOuterPositionIds()[i]].cast<float>();
                }
                else
                {
                    positionsLock->at(i) = positions[i].cast<float>();
                }
            }
        }

        STOP_TIMING_MODELLING;

        START_TIMING_MODELLING("PolygonRenderModel::updatePositions::normals");
        // TODO: vim
        auto normalsLock = mNormalsBufferedData->getData().lock();
        auto facesLock = mFacesBufferedData->getData().lock();

        // TODO: optimize this
        ModelUtils::calculateFaceNormals<float>(
                    *positionsLock,
                    *facesLock,
                    mFaceNormals);

        // calculate normals
        ModelUtils::calculateVertexFromFaceNormals<float>(
                    *positionsLock,
                    *facesLock,
                    mFaceNormals,
                    mPolygon->getTopology(),
                    *normalsLock);

        STOP_TIMING_MODELLING;

//        if (positionsLock->size() != positions.size() ||
//            normalsLock->size() != positions.size() )
//            std::cout << "error: size missmatch\n";
    }
    mPositionsBufferedData->setDataChanged(true);
    mNormalsBufferedData->setDataChanged(true);

    if (mPolygon->getPositionType() == BSWSVectors::BODY_SPACE)
        updateTransform();
}

void PolygonRenderModel::updateNormalLines()
{
    if (!mRenderPoints && !mRenderLinesNormals)
        return;

    float normalLineLength = 0.3f;

    // vertex normals
    if (mRenderVertexNormals)
    {
        auto lines = mRenderLinesNormals->getLines().lock();
        auto points = mRenderPoints->getPoints().lock();

        auto positions = mPositionsBufferedData->getData().lock();
        auto normals = mNormalsBufferedData->getData().lock();

        lines->resize(2 * normals->size());
        points->resize(normals->size());

        Eigen::Affine3f transform = mPolygon->getTransform().cast<float>();

        for (size_t i = 0; i < normals->size(); ++i)
        {
            // calculate center of face
            Vectorf source = transform * positions->at(i);

            // draw line in direction of normal
            Vectorf target =
                    source + normalLineLength * transform.rotation() * normals->at(i);

            (*lines)[2 * i] = source;
            (*lines)[2 * i + 1] = target;

            (*points)[i] = source;
        }
    }

    if (mRenderFaceNormals)
    {
        auto lines = mRenderLinesNormals->getLines().lock();
        auto points = mRenderPoints->getPoints().lock();

        auto positions = mPositionsBufferedData->getData().lock();
        auto facesLock = mFacesBufferedData->getData().lock();

        // face normals
        Vectorfs faceNormals;

        if (mPolygon->getDimensionType() == Polygon::DimensionType::THREE_D)
        {
            Polygon3D* p3 = static_cast<Polygon3D*>(mPolygon.get());
            faceNormals.resize(p3->getOuterFaceNormals().size()); // returns the outer face normals in world space
            for (size_t i = 0; i < faceNormals.size(); ++i)
            {
                faceNormals[i] = p3->getOuterFaceNormals()[i].cast<float>();
            }
        }
        else if (mPolygon->getDimensionType() == Polygon::DimensionType::TWO_D)
        {
            Polygon2D* p2 = static_cast<Polygon2D*>(mPolygon.get());
            faceNormals.resize(p2->getFaceNormals().size());
            for (size_t i = 0; i < faceNormals.size(); ++i)
            {
                faceNormals[i] = p2->getFaceNormals()[i].cast<float>();
            }
        }

//        ModelUtils::calculateFaceNormals<float>(
//                    *positions,
//                    *facesLock,
//                    faceNormals);

        // if mRenderVertexNormals is true, lines and points were already
        // correctly resized in this method and are appended.
        size_t startingIndex;
        if (mRenderVertexNormals)
            startingIndex = points->size();
        else
            startingIndex = 0;

        lines->resize(2 * (startingIndex + faceNormals.size()));
        points->resize(startingIndex + faceNormals.size());

        Eigen::Affine3f transform = mPolygon->getTransform().cast<float>();
        for (size_t i = 0; i < faceNormals.size(); ++i)
        {
            Vectorf pos = Vectorf::Zero();
            for (size_t j = 0; j < 3; ++j)
            {
                pos += positions->at(facesLock->at(i)[j]);
            }
            pos /= 3;

            // calculate center of face
            Vectorf source = transform * pos;

            // draw line in direction of normal
            Vectorf target = source +
                    normalLineLength * faceNormals.at(i);

            (*lines)[2 * (startingIndex + i)] = source;
            (*lines)[2 * (startingIndex + i) + 1] = target;

            (*points)[startingIndex + i] = source;
        }
    }

    mRenderLinesNormals->update();
    mRenderPoints->update();
}

void PolygonRenderModel::updateTransform()
{
    if (mCurrentType == BSWSVectors::Type::BODY_SPACE)
    {
        std::shared_ptr<RenderPolygonsDataBS> dataBS =
                std::static_pointer_cast<RenderPolygonsDataBS>(mRenderPolygonsData);

        Eigen::Affine3d transform = mPolygon->getTransform();
        auto transformLock = dataBS->getTransform().lock();
        *transformLock = transform.cast<float>();

        // set current transform to newly rendered one for future checks.
        mCurrentlyRenderedTransform = transform;
    }
}
