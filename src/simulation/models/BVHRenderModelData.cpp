#include "BVHRenderModelData.h"

BVHRenderModelData::BVHRenderModelData(int level, bool isLeaf)
    : mLevel(level)
    , mIsLeaf(isLeaf)
{

}

BVHRenderModelData::~BVHRenderModelData()
{

}

bool BVHRenderModelData::isToBeSetVisible(bool visible, int level)
{
    if (level == -1) // Render all
    {
        return visible;
    }
    else if (level == -2) // Render leafs
    {
        return visible && mIsLeaf;
    }
    else // Render the corresponding level
    {
        return visible && level == static_cast<int>(mLevel);
    }
}
