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
    SceneData(Node<std::shared_ptr<SceneData>, std::shared_ptr<SceneLeafData>>* node);
    virtual ~SceneData();

    // If you override SceneData, do not call this method from
    // the base class.
    // Simply always execute:
    // sceneDataVisitor->visit(this);
    virtual void accept(SceneDataVisitor* visitor);

    virtual bool isLeafData();

private:

    // not in use
    Eigen::Matrix3d mTransformationMatrix;
};

#endif // SCENEDATA_H
