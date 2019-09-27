#ifndef SCENEDATA_H
#define SCENEDATA_H

// Includes#
#include <Eigen/Dense>
#include <data_structures/tree/Node.h>
#include <data_structures/tree/NodeData.h>
#include <memory>

// Forward declarations
class SceneDataVisitor;
class SceneLeafData;

class SceneData : public NodeData<std::shared_ptr<SceneData>, std::shared_ptr<SceneLeafData>>,
        public std::enable_shared_from_this<SceneData>
{
public:
    enum Visibility
    {
        VISIBLE, INVISIBLE, UNDEFINED
    };

    SceneData(Node<std::shared_ptr<SceneData>, std::shared_ptr<SceneLeafData>>* node);
    virtual ~SceneData();

    // If you override SceneData, do not call this method from
    // the base class.
    // Simply always execute:
    // sceneDataVisitor->visit(this);
    virtual void accept(SceneDataVisitor* visitor);

    virtual bool isLeafData();

    // If this object is a leaf, sets it visible. If its a childrenNode, sets
    // itself and all nodes of its subtree visible.
    virtual void setVisibleRecoursive(bool visible);

    // Retruns the status of visibilty of all nodes of its subtree.
    // - If all are visible or not visible it returns Visibilty::VISIBLE or
    // Visibility::INVISIBLE respectively.
    // - If some are visible and some are not or if there are none at all,
    // then it returns Visibility::UNDEFINED.
    virtual Visibility getSubtreeVisibility();

    bool isSceneDataSelectable() const;
    void setSceneDataSelectable(bool selectable);

    bool isVerticesSelectable() const;
    void setVerticesSelectable(bool selectable);

private:

    // not in use
    Eigen::Matrix3d mTransformationMatrix;

    bool mSceneDataSelectable;
    bool mVerticesSelectable;
};

#endif // SCENEDATA_H
