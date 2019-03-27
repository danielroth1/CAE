#include "BVHCore.h"




BoundingVolume* BVHCore::getBoundingVolume(BVHNode* node)
{
    node->accept(BVHCore::mGetBoundingVolumeVisitor);
    return BVHCore::mGetBoundingVolumeVisitor.boundingVolume;
}


void BVHCore::GetBoundingVolumeVisitor::visit(BVHChildrenNode* childrenNode)
{
    boundingVolume = childrenNode->getData()->getBoundingVolume();
}

void BVHCore::GetBoundingVolumeVisitor::visit(BVHLeafNode* leafNode)
{
    boundingVolume = leafNode->getData()->getBoundingVolume();
}

BVHCore::GetBoundingVolumeVisitor BVHCore::mGetBoundingVolumeVisitor;
