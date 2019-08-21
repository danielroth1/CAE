#include "Selection.h"

#include <data_structures/DataStructures.h>
#include <rendering/ViewFrustum.h>
#include <scene/data/GeometricData.h>
#include <scene/data/GeometricDataVisitor.h>
#include <scene/scene_graph/SceneDataVisitor.h>
#include <scene/scene_graph/SceneLeafData.h>
#include <GL/glu.h>
#include <cmath>
#include <iostream>
#include <scene/data/geometric/GeometricPoint.h>
#include <scene/data/geometric/Polygon2D.h>
#include <scene/data/geometric/Polygon3D.h>

using namespace Eigen;

Selection::Selection()
{
}

Selection::~Selection()
{
}

void Selection::setActive(bool active)
{
    mActive = active;
}

bool Selection::isActive() const
{
    return mActive;
}

//std::set<VertexGroup*>& Selection::getSelectedVertexGroups()
//{
//    return mVertexGroups;
//}
