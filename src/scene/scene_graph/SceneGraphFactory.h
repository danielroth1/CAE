#ifndef SCENEGRAPHFACTORY_H
#define SCENEGRAPHFACTORY_H

#include "SGCore.h"

class SceneGraphFactory
{
public:
    SceneGraphFactory();

    static SGSceneGraph* createDefaultSceneGraph();
};

#endif // SCENEGRAPHFACTORY_H
