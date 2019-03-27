#ifndef TREEFACTORY_CPP
#define TREEFACTORY_CPP

#include "TreeFactory.h"

#include "data_structures/tree/Tree.h"
#include "data_structures/tree/TreeTraverser.h"

template <class T, class L>
Tree<T, L>* TreeFactory<T, L>::createDefautlTree(std::string name)
{
    return new Tree<T, L>(name);
}

template <class T, class L>
TreeTraverser<T, L>* TreeFactory<T, L>::createDefaultTreeTraverser(Tree<T, L>* tree)
{
    return new TreeTraverser<T, L>(tree->getRoot());
}

#endif // TREEFACTORY_CPP
