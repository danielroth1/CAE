
//#include "data_structures/tree/ChildrenNode.h"
//#include "data_structures/tree/LeafNode.h"
//#include "data_structures/tree/Node.h"
//#include "data_structures/tree/Tree.h"
//#include "data_structures/tree/TreeTraverser.h"

//// Factories
//#include "data_structures/tree/factories/TreeFactory.h"
//#include "data_structures/tree/factories/TreeNodeFactory.h"

//// Visitors
//#include "data_structures/tree/NodeVisitor.h"

//// std
//#include <iostream>

//int main(int argc, char *argv[])
//{
//    // Test the scene graph

//    // build some example scene graph

//    // root
//    // A    B
//    // C D  E F
//    // G    H I J

//    typedef TreeFactory<int, int> IntTreeFactory;
//    typedef TreeNodeFactory<int, int> IntTreeNodeFactory;

//    Tree<int, int>* tree = IntTreeFactory::createDefautlTree("Basic Tree");
//    ChildrenNode<int, int>* root = tree->getRoot();

//    ChildrenNode<int, int>* a = IntTreeNodeFactory::createChildrenNode(root, "A");
//    ChildrenNode<int, int>* c = IntTreeNodeFactory::createChildrenNode(a, "C");
//    LeafNode<int, int>* g = IntTreeNodeFactory::createLeafNode(c, "G");
//    LeafNode<int, int>* d = IntTreeNodeFactory::createLeafNode(a);

//    ChildrenNode<int, int>* b = IntTreeNodeFactory::createChildrenNode(root, "B");
//    ChildrenNode<int, int>* e = IntTreeNodeFactory::createChildrenNode(b, "E");
//    LeafNode<int, int>* h = IntTreeNodeFactory::createLeafNode(e, "H");
//    ChildrenNode<int, int>* f = IntTreeNodeFactory::createChildrenNode(b, "F");
//    LeafNode<int, int>* i = IntTreeNodeFactory::createLeafNode(f, "I");
//    LeafNode<int, int>* j = IntTreeNodeFactory::createLeafNode(f, "J");

//    // print tree

//    // define print visitor
//    class PrintVisitor : public NodeVisitor<int, int>{
//    public:
//        virtual void visit(ChildrenNode<int, int>* childrenNode)
//        {
//            std::cout << childrenNode->getName() << "\n";
//        }

//        virtual void visit(LeafNode<int, int>* leafNode)
//        {
//            std::cout << leafNode->getName() << "(" << leafNode->getData() << ")\n";
//        }
//    };

//    // use traverser to iterate tree (not implemented yet)
//    TreeTraverser<int, int>* tt = IntTreeFactory::createDefaultTreeTraverser(tree);
//    PrintVisitor pv;
//    tt->traverse(pv, TraverserStrategy::TOP_TO_BOTTOM);

//    delete tree;

//    return 0;
//}
