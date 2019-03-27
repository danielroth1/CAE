#include "SceneGraphFactory.h"


SceneGraphFactory::SceneGraphFactory()
{

}

SGSceneGraph* SceneGraphFactory::createDefaultSceneGraph()
{

    SGSceneGraph* sg = new SGSceneGraph();
    SGChildrenNode* node = new SGChildrenNode("SimulationObjects");
    sg->getRoot()->addChild(node);

    return sg;
    // physical properties
}
