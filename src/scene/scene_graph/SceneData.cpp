#include "SceneData.h"

#include "SceneDataVisitor.h"
#include "SceneLeafData.h"

#include <scene/scene_graph/SGCore.h>

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

void SceneData::setVisibleRecoursive(bool visible)
{
    if (isLeafData())
    {
        static_cast<SceneLeafData*>(this)->setVisible(visible);
    }
    else
    {
        SGChildrenNode* childrenNode = static_cast<SGChildrenNode*>(getNode());
        for (SGNode* node : childrenNode->getChildren())
        {
            if (node->isLeaf())
            {
                SGLeafNode* leafNode = static_cast<SGLeafNode*>(node);
                leafNode->getData()->setVisible(visible);
            }
            else
            {
                SGChildrenNode* childrenNode = static_cast<SGChildrenNode*>(node);
                childrenNode->getData()->setVisibleRecoursive(visible);
            }
        }
    }
}

SceneData::Visibility SceneData::getSubtreeVisibility()
{
    SGNode* node = getNode();
    if (node->isLeaf())
    {
        return Visibility::UNDEFINED;
    }
    else
    {
        class VisibilityVisitor : public SGNodeVisitor
        {
        public:
            VisibilityVisitor(SGTraverser& _traverser)
                : traverser(_traverser)
            {
                visibilitySet = false;
            }

            virtual void visit(SGChildrenNode* childrenNode) override
            {

            }

            virtual void visit(SGLeafNode* leafNode) override
            {
                if (!visibilitySet)
                {
                    visibilitySet = true;
                    visibility = leafNode->getData()->isVisible() ?
                                Visibility::VISIBLE : Visibility::INVISIBLE;
                }
                else if ((visibility == Visibility::VISIBLE &&
                         !leafNode->getData()->isVisible()) ||
                         (visibility == Visibility::INVISIBLE &&
                          leafNode->getData()->isVisible()))
                {
                    visibility = Visibility::UNDEFINED;
                    traverser.setEndSearchEarly(true);
                }
            }

            bool visibilitySet;
            Visibility visibility;

        private:
            SGTraverser& traverser;
        };

        SGTraverser traverser(node);
        VisibilityVisitor visitor(traverser);
        traverser.traverse(visitor);
        return visitor.visibility;
    }
}
