#ifndef UNIQUEVERTEXID_H
#define UNIQUEVERTEXID_H


#include "data_structures/DataStructures.h"

#include <scene/scene_graph/SceneLeafData.h>

class GeometricData;

class UniqueVertex
{
public:
    UniqueVertex();

    UniqueVertex(SceneLeafData* sceneLeafData, ID vectorID);

    SceneLeafData* getSceneLeafData() const;

    ID getVectorID() const;
    Eigen::Vector& getVector() const;

    //bool operator<(UniqueVertex const& v) const;

private:
    SceneLeafData* mSceneLeafData;
    ID mVectorID;
};

#endif // UNIQUEVERTEXID_H
