#ifndef SELECTION_H
#define SELECTION_H

#include "data_structures/DataStructures.h"
#include "scene/VertexGroup.h"
#include <set>

class UniqueVertex;
class SelectionRectangle;
class Simulation;
class ViewFrustum;

// TODO: not in use currently
enum SelectionPolicy {
    ALL, CLOSEST, PREFER_SELECTED
};

class Selection
{
public:

    Selection();
    virtual ~Selection();

    void setActive(bool active);

    bool isActive() const;

//    std::set<VertexGroup*>& getSelectedVertexGroups();

    // Clears the current selection by calling clear on
    // the vertex collection. Vertex grouops are not removed.
    virtual void clear() = 0;

protected:

//    std::set<VertexGroup*> mVertexGroups;

    bool mActive;
};

#endif // SELECTION_H
