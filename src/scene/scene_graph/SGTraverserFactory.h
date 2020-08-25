#ifndef SGTRAVERSERFACTORY_H
#define SGTRAVERSERFACTORY_H

#include "SGCore.h"

class SGTraverserFactory
{
public:

    static SGTraverser createLeafNodeTraverser(SGNode* root);

    static SGTraverser createDefaultSGTraverser(SGNode* root);

    // Creates a filter that filters out nodes that are visible / invisible.
    // \param filterInvisible - if true, invisible nodes are ignored. Else visible ones are ignored.
    static SGFilter* createVisibilityFilter(bool filterInvisible);

private:
    SGTraverserFactory();
};

#endif // SGTRAVERSERFACTORY_H
