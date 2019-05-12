#include "ModelUtils.h"
#include "PolygonRenderModelImproved.h"

#include <iostream>

#include <scene/data/geometric/GeometricDataListener.h>
#include <scene/data/geometric/GeometricPoint.h>
#include <scene/data/geometric/Polygon.h>
#include <scene/data/geometric/Polygon2D.h>
#include <scene/data/geometric/Polygon2DData.h>
#include <scene/data/geometric/Polygon3D.h>
#include <scene/data/geometric/Polygon3DTopology.h>
#include <scene/data/geometric/PolygonData.h>
#include <scene/data/geometric/TopologyFace.h>

#include <rendering/object/RenderLines.h>
#include <rendering/object/RenderPoints.h>
#include <rendering/object/RenderPolygons.h>
#include <rendering/object/RenderPolygonsConstantData.h>
#include <rendering/object/RenderPolygonsConstantDataBS.h>
#include <rendering/object/RenderPolygonsConstantDataWS.h>
#include <rendering/object/RenderPolygonsData.h>
#include <rendering/object/RenderPolygonsDataBS.h>
#include <rendering/object/RenderPolygonsDataWS.h>

#include <scene/data/GeometricDataVisitor.h>

#include <rendering/Renderer.h>

#include <RenderModelManager.h>

PolygonRenderModelImproved::PolygonRenderModelImproved(
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

    // Resets mRenderObjectPositions and mRenderObjectNormals and resizes them.
    // Initializes mRenderObjectFaces from mPolygon.
    reset();
    updatePositions();

    // Add GeometricDataListener to given GeometricData that checks
    // if the data requires updates. Updates are only performed
    // if it does require them.
    class PolygonListener : public GeometricDataListener
    {
    public:
        PolygonListener(PolygonRenderModelImproved& _model)
            : model(_model)
        {
        }

        virtual void notifyGeometricDataChanged()
        {
            model.mRequiresUpdate = true;
        }

        PolygonRenderModelImproved& model;
    };

    mGeometricListener = std::make_shared<PolygonListener>(*this);
    polygon->addGeometricDataListener(mGeometricListener);
}

PolygonRenderModelImproved::~PolygonRenderModelImproved()
{
    // remove listener that was created in the constructor
    mPolygon->removeGeometricDataListener(mGeometricListener.get());

    if (mRenderPolygons && mRenderPolygonsData)
        mRenderPolygons->removeRenderPolygonsData(mRenderPolygonsData);
}

void PolygonRenderModelImproved::reset()
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
        mRenderPolygonsData = std::make_shared<RenderPolygonsDataBS>();
        break;
    }
    case BSWSVectors::Type::WORLD_SPACE:
    {
        mRenderPolygonsData = std::make_shared<RenderPolygonsDataWS>();
        break;
    }
    }

    mRenderPolygonsData->setColor(Eigen::Vector4f(0.0f, 0.58f, 1.0f, 0.7f)); // teal

    getMonitors(mRenderObjectPositions,
                mRenderObjectNormals,
                mRenderObjectFaces);
    getBufferedData(mPositionsBufferedData,
                    mNormalsBufferedData,
                    mFacesBufferedData);

    // reset positions
    size_t size = mPolygon->getPositions().size();
    mRenderObjectPositions->lock()->resize(size);
    mRenderObjectNormals->lock()->resize(size);

    std::vector<TopologyFace>* faces = retrieveRelevantFaces();
    {
        auto facesLock = mRenderObjectFaces->lock();
        facesLock->resize(faces->size());

        for (size_t i = 0; i < faces->size(); ++i)
        {
            facesLock->at(i) = (*faces)[i].getVertexIds();
        }
    }
    mFacesBufferedData->setDataChanged(true);

    // render normals
    if (mRenderVertexNormals || mRenderFaceNormals)
    {
        mRenderLinesNormals = std::make_shared<RenderLines>();
        mRenderPoints = std::make_shared<RenderPoints>();
    }

    updatePositions();

    if (replacedRenderPolygons)
        mAddedToRenderer = false;

}

void PolygonRenderModelImproved::update()
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

void PolygonRenderModelImproved::revalidate()
{
    if (mRenderPolygons->getType() == BSWSVectors::Type::BODY_SPACE)
        std::static_pointer_cast<RenderPolygonsDataBS>(mRenderPolygonsData)->getTransform()->setIdentity();

    updatePositions();
}

void PolygonRenderModelImproved::accept(RenderModelVisitor& /*v*/)
{

}

void PolygonRenderModelImproved::addToRenderer(Renderer* renderer)
{
    mRenderPolygons->setDomain(renderer->getDomain());
    mRenderPolygons->addRenderPolygonsData(mRenderPolygonsData);
    renderer->addRenderObject(mRenderPolygons);
//    renderer->addRenderObject(mRenderPolygonsData);

    if (mRenderLinesNormals)
        renderer->addRenderObject(mRenderLinesNormals);

    if (mRenderPoints)
        renderer->addRenderObject(mRenderPoints);
}

void PolygonRenderModelImproved::removeFromRenderer(Renderer* renderer)
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

void PolygonRenderModelImproved::setVisible(bool visible)
{
    mRenderPolygonsData->setVisible(visible);
    if (mRenderVertexNormals)
    {
        mRenderLinesNormals->setVisible(visible);
        mRenderPoints->setVisible(visible);
    }
    RenderModel::setVisible(visible);
}

std::vector<TopologyFace>* PolygonRenderModelImproved::retrieveRelevantFaces()
{
    class GetFacesVisitor : public GeometricDataVisitor
    {
    public:
        GetFacesVisitor(PolygonRenderModelImproved& _rm)
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

        PolygonRenderModelImproved& rm;
        std::vector<TopologyFace>* faces;
    } visitor(*this);

    mPolygon->accept(visitor);
    return visitor.faces;
}

void PolygonRenderModelImproved::getMonitors(
        Monitor<Vectorfs>*& positionsMonitor,
        Monitor<Vectorfs>*& normalsMonitor,
        Monitor<Faces>*& facesMonitor)
{
    switch(mRenderPolygons->getType())
    {
    case BSWSVectors::BODY_SPACE:
    {
        std::shared_ptr<RenderPolygonsDataBS> dataBS =
                std::static_pointer_cast<RenderPolygonsDataBS>(mRenderPolygonsData);
        std::shared_ptr<RenderPolygonsConstantDataBS> constantDataBS =
                std::static_pointer_cast<RenderPolygonsConstantDataBS>(mRenderPolygons->getConstantData());

        positionsMonitor = &constantDataBS->getPositionsBuffer().getData();
        normalsMonitor = &constantDataBS->getNormalsBuffer().getData();
        facesMonitor = &constantDataBS->getFacesBuffer().getData();

        break;
    }
    case BSWSVectors::WORLD_SPACE:
    {
        std::shared_ptr<RenderPolygonsDataWS> dataWS =
                std::static_pointer_cast<RenderPolygonsDataWS>(mRenderPolygonsData);
        std::shared_ptr<RenderPolygonsConstantDataWS> constantDataWS =
                std::static_pointer_cast<RenderPolygonsConstantDataWS>(mRenderPolygons->getConstantData());

        positionsMonitor = &dataWS->getPositionsBuffer().getData();
        normalsMonitor = &dataWS->getNormalsBuffer().getData();
        facesMonitor = &constantDataWS->getFacesBuffer().getData();

        break;
    }
    }
}

void PolygonRenderModelImproved::getBufferedData(
        BufferedData<Vectorf, float, 3>*& positionsBufferedData,
        BufferedData<Vectorf, float, 3>*& normalsBufferedData,
        BufferedData<Face, unsigned int, 3>*& facesBufferedData)
{
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
        positionsBufferedData = &constantDataBS->getPositionsBuffer();
        normalsBufferedData = &constantDataBS->getNormalsBuffer();
        facesBufferedData = &constantDataBS->getFacesBuffer();

        break;
    }
    case BSWSVectors::WORLD_SPACE:
    {
        std::shared_ptr<RenderPolygonsDataWS> dataWS =
                std::static_pointer_cast<RenderPolygonsDataWS>(mRenderPolygonsData);
        std::shared_ptr<RenderPolygonsConstantDataWS> constantDataWS =
                std::static_pointer_cast<RenderPolygonsConstantDataWS>(mRenderPolygons->getConstantData());

        positionsBufferedData = &dataWS->getPositionsBuffer();
        normalsBufferedData = &dataWS->getNormalsBuffer();
        facesBufferedData = &constantDataWS->getFacesBuffer();

        break;
    }
    }
}

void PolygonRenderModelImproved::updatePositions()
{
    Vectors& positions
            = mPolygon->getPositionType() == BSWSVectors::BODY_SPACE ?
                mPolygon->getPositionsBS() :
                mPolygon->getPositions(); // for 3d case, this are all 3d positions, but often only 2d positions are required

    {
        auto positionsLock = mRenderObjectPositions->lock();
        for (size_t i = 0; i < positionsLock->size(); ++i)
        {
            const Vector& v = positions[i];
            positionsLock->at(i) = v.cast<float>();
        }

        auto normalsLock = mRenderObjectNormals->lock();
        auto facesLock = mRenderObjectFaces->lock();
        // calculate normals
        ModelUtils::calculateNormals<float>(
                    *positionsLock,
                    *facesLock,
                    *normalsLock);
    }
    mPositionsBufferedData->setDataChanged(true);
    mNormalsBufferedData->setDataChanged(true);

    if (mPolygon->getPositionType() == BSWSVectors::BODY_SPACE)
        updateTransform();
}

void PolygonRenderModelImproved::updateNormalLines()
{
    if (!mRenderVertexNormals && !mRenderLinesNormals)
        return;

    float normalLineLength = 0.3f;

    // vertex normals
    if (mRenderVertexNormals)
    {
        auto lines = mRenderLinesNormals->getLines().lock();
        auto points = mRenderPoints->getPoints().lock();

        auto positions = mRenderObjectPositions->lock();
        auto normals = mRenderObjectNormals->lock();

        lines->resize(2 * normals->size());
        points->resize(normals->size());

        Eigen::Affine3f transform = mPolygon->getTransform().cast<float>();

        for (size_t i = 0; i < normals->size(); ++i)
        {    
            // calculate center of face
            Vectorf source = transform * positions->at(i);

            // draw line in direction of normal
            Vectorf target = source + normalLineLength * normals->at(i);

            (*lines)[2 * i] = source;
            (*lines)[2 * i + 1] = target;

            (*points)[i] = source;
        }
    }

    if (mRenderFaceNormals)
    {
        auto lines = mRenderLinesNormals->getLines().lock();
        auto points = mRenderPoints->getPoints().lock();

        auto positions = mRenderObjectPositions->lock();
        auto facesLock = mRenderObjectFaces->lock();

        // face normals
        Vectorfs faceNormals;
        ModelUtils::calculateFaceNormals<float>(
                    *positions,
                    *facesLock,
                    faceNormals);

        // if mRenderVertexNormals is true, lines and points were already
        // correctly resized in this method and are appended.
        size_t startingIndex;
        if (mRenderVertexNormals)
            startingIndex = lines->size();
        else
            startingIndex = 0;

        lines->resize(startingIndex + 2 * faceNormals.size());
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
                    normalLineLength * transform.linear() * faceNormals.at(i);

            (*lines)[startingIndex + 2 * i] = source;
            (*lines)[startingIndex + 2 * i + 1] = target;

            (*points)[startingIndex + i] = source;
        }
    }

    mRenderLinesNormals->update();
    mRenderPoints->update();
}

void PolygonRenderModelImproved::updateTransform()
{
    Eigen::Affine3d transform = mPolygon->getTransform();

    std::shared_ptr<RenderPolygonsDataBS> dataBS =
            std::static_pointer_cast<RenderPolygonsDataBS>(mRenderPolygonsData);
    auto transformLock = dataBS->getTransform().lock();
    *transformLock = transform.cast<float>();

    // set current transform to newly rendered one for future checks.
    mCurrentlyRenderedTransform = transform;
}
