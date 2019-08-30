#ifndef SGCONTROL_H
#define SGCONTROL_H

#include <scene/scene_graph/SGCore.h>

#include <SimulationControl.h>

#include <io/File.h>

class ApplicationControl;
class MeshCriteria;

// Manages a SGSceneGraph
// The scnee graph should only be accessed over this class to ensure
// consistency, especially in the case that the scene graph is removed.
// Not all control duties are implemented here. Events that adapt the
// scene graph can be caught with a SGTreeListener.
class SGControl : public SGTreeListener
{
public:
    SGControl();

    // initializes everything that may depend on other control
    // classes.
    void initialize(ApplicationControl* ac);

    // Removes all elements from the scene graph (except the root node).
    // Removes all simulation objects from the simnulation.
    void clearScene();

    // SG Insertion Methods
        // Imports the file at the specified path and stores a new
        // node to the provided children node.
        SGNode* importFileAsChild(
                File file,
                SGChildrenNode* childrenNode,
                bool renderOnlyOuterFaces = true);

        // Creates a sphere
        // \param name - name of the node
        // \param parent - parent node of the created node
        SGLeafNode* createBox(
                std::string name,
                SGChildrenNode* parent,
                Eigen::Vector position,
                double width,
                double height,
                double length,
                bool renderOnlyOuterFaces = true);

        // Creates a 2d box
        // \param name - name of the node
        // \param parent - parent node of the created node
        SGLeafNode* createSphere(
                std::string name,
                SGChildrenNode* parent,
                Eigen::Vector position,
                double radius,
                int resolution = 3,
                bool renderOnlyOuterFaces = true);

        // Returns the given leaf node.
        SGLeafNode* create3DGeometryFrom2D(
                SGLeafNode* leafNode,
                const MeshCriteria& meshCriteria,
                bool renderOnlyOuterFaces = true);

        SGLeafNode* createSimulationPoint(
                std::string name,
                SGChildrenNode* parent,
                Eigen::Vector position);

        // Creates a linear force from a SimulationPointRef to a target point.
        // Creates the point (GeometricPoint and SimulationPoint) and adds
        // them to parent.
        void createLinearForce(
                std::string name,
                SGChildrenNode* parent,
                SimulationPointRef source,
                const Eigen::Vector& target,
                double strength);

    // Simulation Methods
        std::shared_ptr<FEMObject> createFEMObject(
                const std::shared_ptr<SceneLeafData>& ld);
        std::shared_ptr<RigidBody> createRigidBody(
                const std::shared_ptr<SceneLeafData>& ld,
                double mass,
                bool iStatic = false);
        void createCollidable(const std::shared_ptr<SceneLeafData>& ld,
                              double collisionSphereRadiusFactor = 0.1);


    // Factory methods

        SGChildrenNode* createChildrenNode(
                SGChildrenNode* parent,
                std::string name);

        // Creates a leaf node, adds it to the root node of the scene
        // graph and returns it.
        SGLeafNode* createAndAddLeafNodeToRoot(
                std::string name = "");

        // Creates a leaf node, adds it to the node that shares the
        // provided node name
        SGLeafNode* createAndAddLeafNode(
                std::string parentName,
                std::string childName = "");

        // Retrieves the geometric data from the leaf node,
        // creates the corresponding simulation object, and
        // adds it to the leaf node.
        // This method allows to simply add any kind of node
        // to a simulation.
        void createAndSetCorrespondingSimulationObject(
                SGLeafNode* leafNode);

        // TODO: In the future rules should be specified which
        // kind of simulation object should be created from
        // the provided geometric data, but for now there are
        // always good 1:1 relationships between geometric and
        // simulation data.
        SimulationObject* createCorrespondingSimulationObject(
                GeometricData* geo);

        SGTraverser createSceneGraphTraverser();

        void removeNode(SGNode* node);

    // Scene graph insertion methods

        // Creates a leaf node with the given name and parent.
        // Inserts to its leaf data the given GeometricData.
        //\param name - name of the created node
        //\param parent - parent node
        SGLeafNode* createLeafNode(
                std::string name,
                SGChildrenNode* parent,
                std::shared_ptr<Polygon> polygon,
                Eigen::Vector position = Eigen::Vector::Zero(),
                bool renderOnlyOuterFaces = true);

        SGLeafNode* createLeafNode(
                std::string name,
                SGChildrenNode* parent,
                std::shared_ptr<Polygon> polygon,
                Eigen::Affine3d transform,
                bool renderOnlyOuterFaces = true);

    // Getters
        SGSceneGraph* getSceneGraph();

        SGNode* getSceneNodeByName(std::string name);

    // LeafNodeListener interface
    public:
        virtual void notifyLeafDataChanged(SGNode* source,
                                           std::shared_ptr<SceneLeafData>& data);

    // ChildrenNodeListener interface
    public:
        virtual void notifyChildAdded(SGNode* source, SGNode* childNode);
        virtual void notifyChildRemoved(SGNode* source, SGNode* childNode);
        virtual void notifyChildrenDataChanged(SGNode* source,
                                               std::shared_ptr<SceneData>& data);

    // NodeListener interface
    public:
        virtual void notifyParentChanged(SGNode* source, SGNode* parent);
        virtual void notifyNameChanged(SGNode* source, std::string name);
        virtual void notifyTreeChanged(SGNode* source,
                                       Tree<std::shared_ptr<SceneData>,
                                       std::shared_ptr<SceneLeafData>>* tree);

private:

    ApplicationControl* mAc;
    SGSceneGraph* mSceneGraph;


};

#endif // SGCONTROL_H
