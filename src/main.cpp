

#include <modules/mesh_converter/MeshConverter.h>
#include <SimulationControl.h>

#include "main_window.h"

#include <QApplication>
#ifdef WIN32
#include <windows.h>
#endif
#include <GL/glut.h>
#include <glwidget.h>
#include <iostream>
#include <fstream>
//#include <fmi/FmiImporter.h>

#include <atomic>
#include <thread>
#include <chrono>

#include <QDebug>

#include "SimulationControl.h"
#include "ui/UIControl.h"
#include "ApplicationControl.h"

int mainSceneGraph(int argc, char *argv[]);

int main(int argc, char *argv[])
{
    // FMI

//    FmiImporter importer;
//    importer.executeTest(argc, argv);
//    return 0;


    // Scene Graph

//    return mainSceneGraph(argc, argv);


    // Standard Application

    glutInit(&argc, argv);

    qputenv("QT_STYLE_OVERRIDE","");
    QApplication a(argc, argv);

//    QSurfaceFormat format;
//    format.setDepthBufferSize(24);
//    format.setStencilBufferSize(8);
//    format.setVersion(3, 2);
//    format.setProfile(QSurfaceFormat::CoreProfile);
//    QSurfaceFormat::setDefaultFormat(format);

    ApplicationControl ac;
    ac.initiateApplication();

    return a.exec();
}


//#include "data_structures/tree/ChildrenNode.h"
//#include "data_structures/tree/LeafNode.h"
//#include "data_structures/tree/Node.h"
//#include "data_structures/tree/TraversalFilter.h"
//#include "data_structures/tree/Tree.h"
//#include "data_structures/tree/TreeTraverser.h"

//// Factories
//#include "data_structures/tree/factories/TreeFactory.h"
//#include "data_structures/tree/factories/TreeNodeFactory.h"

//// Visitors
//#include "data_structures/tree/NodeVisitor.h"

//// std
//#include <iostream>

//int mainSceneGraph(int /*argc*/, char */*argv*/[])
//{
//    // Test the scene graph

//    // build some example scene graph

//    // root
//    // A    B
//    // C D  E F
//    // G    H I J

//    typedef TreeFactory<int, int> TreeFactoryInt;
//    typedef TreeNodeFactory<int, int> TreeNodeFactoryInt;

//    typedef Node<int, int> Node;
//    typedef ChildrenNode<int, int> ChildrenNode;
//    typedef LeafNode<int, int> LeafNode;

//    Tree<int, int>* tree = TreeFactoryInt::createDefautlTree("Basic Tree");
//    ChildrenNode* root = tree->getRoot();

//    ChildrenNode* a = TreeNodeFactoryInt::createChildrenNode(root, "A");
//    ChildrenNode* c = TreeNodeFactoryInt::createChildrenNode(a, "C");
//    TreeNodeFactoryInt::createLeafNode(c, "G");
//    TreeNodeFactoryInt::createLeafNode(a, "D");

//    ChildrenNode* b = TreeNodeFactoryInt::createChildrenNode(root, "B");
//    ChildrenNode* e = TreeNodeFactoryInt::createChildrenNode(b, "E");
//    TreeNodeFactoryInt::createLeafNode(e, "H");
//    ChildrenNode* f = TreeNodeFactoryInt::createChildrenNode(b, "F");
//    TreeNodeFactoryInt::createLeafNode(f, "I");
//    TreeNodeFactoryInt::createLeafNode(f, "J");

//    // print tree

//    // define print visitor
//    class PrintVisitor : public NodeVisitor<int, int>{
//    public:

//        PrintVisitor(TreeTraverser<int, int>& _tt)
//            : tt(_tt)
//        {
//        }

//        virtual void visit(ChildrenNode* childrenNode)
//        {
//            for (size_t i = 0; i < tt.getCurrentLevel(); ++i)
//                std::cout << "...";
//            std::cout << childrenNode->getName() << "\n";
//        }

//        virtual void visit(LeafNode* leafNode)
//        {
//            for (size_t i = 0; i < tt.getCurrentLevel(); ++i)
//                std::cout << "...";
//            std::cout << leafNode->getName() << "(" << leafNode->getData() << ")\n";
//        }

//        TreeTraverser<int, int>& tt;
//    };

//    class LeafFilter : public TraversalFilter<int, int>, public NodeVisitor<int, int>
//    {
//        bool filter(Node* data)
//        {
//            data->accept(*this);
//            return doFilter;
//        }

//        // NodeVisitor<int>
//        void visit(ChildrenNode* /*childrenNode*/)
//        {
//            doFilter = false;
//        }

//        void visit(LeafNode* /*leafNode*/)
//        {
//            doFilter = true;
//        }

//        bool doFilter;
//    };

//    // use traverser to iterate tree (not implemented yet)
//    TreeTraverser<int, int>* tt = TreeFactoryInt::createDefaultTreeTraverser(tree);
////    LeafFilter ff;
////    tt->setFilter(&ff);
//    PrintVisitor pv(*tt);

//    std::cout << "TOP_TO_BOTTOM:\n";
//    tt->traverse(pv, TraverserStrategy::TOP_TO_BOTTOM);
//    std::cout << "\nBOTTOM_TO_TOP:\n";
//    tt->traverse(pv, TraverserStrategy::BOTTOM_TO_TOP);

//    delete tree;

//    return 0;
//}
