#pragma once

#ifndef CHILDRENNODE_H
#define CHILDRENNODE_H

// Inlcudes
#include "Node.h"
#include <vector>

// Forward declarations
template <class T, class L>
class NodeVisitor;

template <class T, class L>
class ChildrenNodeListener;

template <class T, class L>
class ChildrenNode : public Node<T, L>
{
public:
    ChildrenNode(std::string name = "");
    ChildrenNode(ChildrenNode* parent, std::string name = "");
    virtual ~ChildrenNode();

    // Delegated children methods =======
        Node<T, L>* getChild(unsigned int i) const;

        // Returns the children nodes.
        const std::vector<Node<T, L>*>& getChildren();

        // Adds a child. Sets the parent of the child to this
        // node. Do not call this method with nodes that are
        // already added to a tree.
        void addChild(Node<T, L>* node);
        void removeChild(Node<T, L>* node);
        // Removes all children.
        void clear();
        virtual std::size_t getNumberOfChildren() const;
        typename std::vector<Node<T, L>*>::iterator begin();
        typename std::vector<Node<T, L>*>::iterator end();
    // ===================================

    // Tree methods ======================
        virtual void registerTree(Tree<T, L>* tree);
    // ===================================

    // Visitor
        void accept(NodeVisitor<T, L>& visitor);

    // Setters
        virtual void setData(T data);
        virtual void setTree(Tree<T, L>* tree);

    // Getters
        virtual T getData() const;
        virtual bool isLeaf() const;

    // Listener Support
        virtual void addListener(ChildrenNodeListener<T, L>* listener);
        virtual void removeListener(ChildrenNodeListener<T, L>* listener);

protected:

    std::vector<Node<T, L>*> mChildren;

    std::vector<ChildrenNodeListener<T, L>*> mListeners;

    T mData;
};

#include "ChildrenNode.cpp"

#endif // CHILDRENNODE_H
