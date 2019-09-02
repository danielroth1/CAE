
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

void GeometricData::update()
{
    for (const auto& listener : mListeners)
        listener->notifyGeometricDataChanged();
}

void GeometricData::addGeometricDataListener(std::shared_ptr<GeometricDataListener> listener)
{
    mListeners.push_back(listener);
}

bool GeometricData::removeGeometricDataListener(GeometricDataListener* listener)
{
    auto it = std::find_if(mListeners.begin(), mListeners.end(),
                           [&listener](const std::shared_ptr<GeometricDataListener>& l)
    {
        return l.get() == listener;
    });
    if (it != mListeners.end())
    {
        mListeners.erase(it);
        return true;
    }
    return false;
}

BoundingBox& GeometricData::getBoundingBox()
{
    return mBoundingBox;
}
