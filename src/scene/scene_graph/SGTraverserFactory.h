#ifndef SGTRAVERSERFACTORY_H
#define SGTRAVERSERFACTORY_H

#include "SGCore.h"

class SGTraverserFactory
{
public:

    static SGTraverser createLeafNodeTraverser(SGNode* root);

    static SGTraverser createDefaultSGTraverser(SGNode* root);

private:
    SGTraverserFactory();
};

#endif // SGTRAVERSERFACTORY_H
