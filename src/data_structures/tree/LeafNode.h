#ifndef LEAFNODE_H
#define LEAFNODE_H

// Includes
//#include "ChildrenNode.h"
#include "Node.h"

//#include "NodeVisitor.h"

// Forware declarations
template <class T, class L>
class NodeVisitor;

template <class T, class L>
class ChildrenNode;

template <class T, class L>
class LeafNodeListener;

template <class T, class L>
class LeafNode : public Node<T, L>
{
public:
    // Constructors and Destructors =========
        LeafNode<T, L>(std::string name);
        LeafNode<T, L>(ChildrenNode<T, L>* parent, std::string name);
        virtual ~LeafNode<T, L>();
    // ======================================

    // Node Visitor Pattern =================
        virtual void accept(NodeVisitor<T, L>& visitor);
    // ======================================

    // Tree methods ======================
        virtual void registerTree(Tree<T, L>* tree);
    // ===================================

    // Getters
        L& getData();
        virtual std::size_t getNumberOfChildren() const;

    // Setters
        void setTree(Tree<T, L>* tree);
        void setData(L data);

    // Listener Support
        void addListener(LeafNodeListener<T, L>* listener);
        void removeListener(LeafNodeListener<T, L>* listener);

protected:
    L mData;

    std::vector<LeafNodeListener<T, L>*> mListeners;

};

#include "LeafNode.cpp"

#endif // LEAFNODE_H
