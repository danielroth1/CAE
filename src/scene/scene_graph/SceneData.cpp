#include "SceneData.h"

#include "SceneDataVisitor.h"
#include "SceneLeafData.h"

SceneData::SceneData(Node<std::shared_ptr<SceneData>, std::shared_ptr<SceneLeafData>>* node)
    : NodeData<std::shared_ptr<SceneData>, std::shared_ptr<SceneLeafData>>(node)
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
