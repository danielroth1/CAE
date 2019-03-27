#include "SceneData.h"

#include "SceneDataVisitor.h"
#include "SceneLeafData.h"

SceneData::SceneData(Node<SceneData*, SceneLeafData*>* node)
    : NodeData<SceneData*, SceneLeafData*>(node)
{

}

SceneData::~SceneData()
{

}

void SceneData::accept(SceneDataVisitor* visitor)
{
    visitor->visit(this);
}

bool SceneData::isLeafData()
{
    return false;
}
