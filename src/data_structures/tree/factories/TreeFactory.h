#ifndef TREEFACTORY_H
#define TREEFACTORY_H

// Includes
//#include "data_structures/tree/Tree.h"
//#include "data_structures/tree/TreeTraverser.h"
#include <string>

// Forward Declarations
template <class T, class L>
class Tree;

template <class T, class L>
class TreeTraverser;

template <class T, class L>
class TreeFactory
{
public:
    static Tree<T, L>* createDefautlTree(std::string name = "");

    static TreeTraverser<T, L>* createDefaultTreeTraverser(Tree<T, L>* tree);

private:
    TreeFactory();
};

#include "TreeFactory.cpp"

#endif // TREEFACTORY_H
