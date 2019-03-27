#include "ModelUtils.h"
#include "PolygonRenderModel.h"

#include <scene/data/geometric/GeometricDataListener.h>
#include <scene/data/geometric/Polygon.h>
#include <scene/data/geometric/Polygon2D.h>
#include <scene/data/geometric/Polygon3D.h>

#include <rendering/object/RenderLines.h>
#include <rendering/object/RenderPoints.h>
#include <rendering/object/RenderPolygon2D.h>

#include <rendering/Renderer.h>

#include <scene/data/GeometricDataVisitor.h>

#include <iostream>

PolygonRenderModel::PolygonRenderModel(
        std::shared_ptr<Polygon> polygon,
        bool renderOnlyOuterFaces)
    : mPolygon(polygon)
    , mRenderOnlyOuterFaces(renderOnlyOuterFaces)
{
    mRenderPoly2 = nullptr;
    mRequiresUpdate = false;
    mCurrentlyRenderedCenter = Vector::Zero();

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
}

void PolygonRenderModel::reset()
{
    // Init render object
    mRenderPoly2 = std::make_shared<RenderPolygon2D>();

    mRenderObjectPositions = &mRenderPoly2->getPositions();
    mRenderObjectNormals = &mRenderPoly2->getNormals();
    mRenderObjectFaces = &mRenderPoly2->getFaces();

    // reset positions
    size_t size = mPolygon->getPositions().size();
    mRenderObjectPositions->lock()->resize(size);
    mRenderObjectNormals->lock()->resize(size);

    Faces* faces = retrieveRelevantFaces();

    auto facesLock = mRenderObjectFaces->lock();
    facesLock->resize(faces->size());

    for (size_t i = 0; i < faces->size(); ++i)
    {
        facesLock->at(i) = (*faces)[i];
    }

    // render normals
    mRenderLinesNormals = std::make_shared<RenderLines>();
    mRenderPoints = std::make_shared<RenderPoints>();

}

void PolygonRenderModel::update()
{
    // update the transformation matrix if it changed by a certain degree

    if (!mRequiresUpdate)
        return;

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
        Eigen::Affine3d transform = mPolygon->getTransform();
        if (!mCurrentlyRenderedTransform.isApprox(transform, 1e-3))
        {
            // Render transform
            mRenderPoly2->getTransform() = transform.cast<float>();

            // set current transform to newly rendered one for future checks.
            mCurrentlyRenderedTransform = transform;
        }

        break;
    }
    case BSWSVectors::WORLD_SPACE:
        updatePositions();
        break;
    }

    updateNormalLines();
}

void PolygonRenderModel::revalidate()
{
    mRenderPoly2->getTransform().setIdentity();
    updatePositions();
}

void PolygonRenderModel::accept(RenderModelVisitor& /*v*/)
{

}

void PolygonRenderModel::addToRenderer(Renderer* renderer)
{
    renderer->addRenderObject(mRenderPoly2);
//    renderer->addRenderObject(mRenderLinesNormals);
//    renderer->addRenderObject(mRenderPoints);
}

void PolygonRenderModel::removeFromRenderer(Renderer* renderer)
{
    renderer->removeRenderObject(mRenderPoly2);
    renderer->removeRenderObject(mRenderLinesNormals);
    renderer->removeRenderObject(mRenderPoints);
}

void PolygonRenderModel::setVisible(bool visible)
{
    mRenderPoly2->setVisible(visible);
    mRenderLinesNormals->setVisible(visible);
    mRenderPoints->setVisible(visible);
    RenderModel::setVisible(visible);
}

Faces* PolygonRenderModel::retrieveRelevantFaces()
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
            faces = &polygon2D.getFaces();
        }

        virtual void visit(Polygon3D& polygon3D)
        {
            if (rm.mRenderOnlyOuterFaces)
                faces = &polygon3D.getOuterFaces();
            else
                faces = &polygon3D.getFaces();
        }

        virtual void visit(GeometricPoint& /*point*/)
        {

        }

        PolygonRenderModel& rm;
        Faces* faces;
    } visitor(*this);

    mPolygon->accept(visitor);
    return visitor.faces;
}

void PolygonRenderModel::updatePositions()
{
    Vectors& positions
            = mPolygon->getPositionType() == BSWSVectors::BODY_SPACE ?
                mPolygon->getPositionsBS() :
                mPolygon->getPositions();

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

void PolygonRenderModel::updateNormalLines()
{
    // Lines
    {
        auto lines = mRenderLinesNormals->getLines().lock();
        auto points = mRenderPoints->getPoints().lock();

        auto positions = mRenderObjectPositions->lock();
        auto normals = mRenderObjectNormals->lock();

        lines->resize(2 * normals->size());
        points->resize(normals->size());

        float normalLineLength = 0.1f;

        for (size_t i = 0; i < normals->size(); ++i)
        {
            // calculate center of face
            Vectorf source = positions->at(i);

            // draw line in direction of normal
            Vectorf target = source + normalLineLength * normals->at(i);

            (*lines)[2 * i] = source;
            (*lines)[2 * i + 1] = target;

            (*points)[i] = source;
        }
    }

    mRenderLinesNormals->update();
}
