#ifndef SGCONTROL_H
#define SGCONTROL_H

#include <scene/scene_graph/SGCore.h>

#include <SimulationControl.h>

class ApplicationControl;

// Manages a SGSceneGraph
// The scnee graph should only be accessed over this class to ensure
// consistency, especially in the case that the scene graph is removed.
// Not all control duties are implemented here. Events that adapt the
// scene graph can be caught with a SGTreeListener.
class SGControl
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
        SGLeafNode* importFileAsChild(
                std::string path,
                std::string name,
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
                double cellSize = 0.3,
                double cellRadiusEdgeRatio = 30,
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
        void createFEMObject(const std::shared_ptr<SceneLeafData>& ld);
        void createRigidBody(const std::shared_ptr<SceneLeafData>& ld,
                             double mass,
                             bool iStatic = false);
        void createCollidable(const std::shared_ptr<SceneLeafData>& ld);


    // Factory methods
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


    // Getters
        SGSceneGraph* getSceneGraph();

        SGNode* getSceneNodeByName(std::string name);

private:

        // Scene graph insertion methods

        // Creates a leaf node with the given name and parent.
        // Inserts to its leaf data the given GeometricData.
        //\param name - name of the created node
        //\param parent - parent node
        SGLeafNode* createLeafNode(
                std::string name,
                SGChildrenNode* parent,
                std::shared_ptr<Polygon> geoData,
                bool renderOnlyOuterFaces);

    ApplicationControl* mAc;
    SGSceneGraph* mSceneGraph;
};

#endif // SGCONTROL_H
