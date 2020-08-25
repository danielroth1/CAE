#include "SGTraverserFactory.h"


SGTraverser
SGTraverserFactory::createLeafNodeTraverser(SGNode* root)
{
    class LeafNodeFilter : public SGFilter
    {
        bool filter(SGNode* sceneNode)
        {
            class LeafVisitor : public SGNodeVisitor
            {
            public:
                void visit(SGChildrenNode*)
                {
                    isLeaf = false;
                }
                void visit(SGLeafNode*)
                {
                    isLeaf = true;
                }
                bool isLeaf;
            } v;
            sceneNode->accept(v);
            return v.isLeaf;
        }
    };
    return SGTraverser(root, new LeafNodeFilter());
}

SGTraverser SGTraverserFactory::createDefaultSGTraverser(SGNode* root)
{
    return SGTraverser(root, nullptr);
}

SGFilter* SGTraverserFactory::createVisibilityFilter(bool filterInvisible)
{
    class VisibilityFilter : public SGFilter
    {
    public:
        VisibilityFilter(bool _filterInvisible)
            : filterInvisible(_filterInvisible)
        {

        }

        bool filter(SGNode* sceneNode)
        {
            if (sceneNode->isLeaf())
            {
                bool visible = static_cast<SGLeafNode*>(sceneNode)->getData()->isVisible();
                if (filterInvisible)
                    return !visible;
                else
                    return visible;
            }
            return false;
        }
        bool filterInvisible;
    };
    return new VisibilityFilter(filterInvisible);
}

SGTraverserFactory::SGTraverserFactory()
{

}
