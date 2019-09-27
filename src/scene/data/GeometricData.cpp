
// Includes
#include "GeometricData.h"
#include "GeometricDataVisitor.h"

#include <scene/data/geometric/GeometricDataListener.h>
#include <scene/data/geometric/PositionData.h>



using namespace Eigen;


GeometricData::GeometricData()
{
}

GeometricData::~GeometricData()
{

}

GeometricData::GeometricData(const GeometricData& gd)
    : std::enable_shared_from_this<GeometricData> ()
{
    mBoundingBox = gd.mBoundingBox;
}

void GeometricData::update(bool /*updateFaceNormals*/, bool /*updateVertexNormals*/)
{
    auto listeners = mListeners.lock();
    for (std::shared_ptr<GeometricDataListener> listener : *listeners)
        listener->notifyGeometricDataChanged();
}

void GeometricData::addGeometricDataListener(std::shared_ptr<GeometricDataListener> listener)
{
    auto listeners = mListeners.lock();
    listeners->push_back(listener);
}

bool GeometricData::removeGeometricDataListener(GeometricDataListener* listener)
{
    auto listeners = mListeners.lock();
    auto it = std::find_if(listeners->begin(), listeners->end(),
                           [&listener](const std::shared_ptr<GeometricDataListener>& l)
    {
        return l.get() == listener;
    });
    if (it != listeners->end())
    {
        listeners->erase(it);
        return true;
    }
    return false;
}

BoundingBox& GeometricData::getBoundingBox()
{
    return mBoundingBox;
}
