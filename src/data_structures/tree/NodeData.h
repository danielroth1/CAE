#ifndef NODEDATA_H
#define NODEDATA_H

#include "Node.h"

// This class can be used if it is necessary to access children nodes
// from the classes that are used as templates. Those must inherit from
// this class and the node that they are added to must be provided in
// the constructor.
//
// Also since getChildData always returns a pointer to T* it is assumed
// that T == L.
template <class T, class L>
class NodeData
{
public:
    // Constructor
        NodeData(Node<T, L>* node);

        Node<T, L>* getNode();

    // Delegated ChildrenNode Methods

        // \return - number of children. 0 if the node is a leaf node.
        std::size_t getNumberOfChildren() const;

private:
    // Private NodeVisitorClass

        class NodeGetNumberOfChildrenVisitor : public NodeVisitor<T, L>
        {
        public:
            virtual void visit(ChildrenNode<T, L>* childrenNode)
            {
                returnValue = childrenNode->getNumberOfChildren();
            }

            virtual void visit(LeafNode<T, L>* /*leafNode*/)
            {
                returnValue = 0;
            }

            unsigned int returnValue;
        };


    // Private Members
        Node<T, L>* mNode;
        NodeGetNumberOfChildrenVisitor mGetNumberOfChildrenVisitor;
};

#include "NodeData.cpp"

#endif // NODEDATA_H
