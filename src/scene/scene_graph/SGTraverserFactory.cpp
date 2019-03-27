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

SGTraverserFactory::SGTraverserFactory()
{

}
