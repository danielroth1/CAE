#ifndef SGCONTROL_H
#define SGCONTROL_H

#include <scene/scene_graph/SGCore.h>

#include <SimulationControl.h>

#include <io/File.h>

class ApplicationControl;
class MeshCriteria;
class MeshInterpolationMeshMesh;

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

    // Casts a ray from origin along the given normal and checks with which
    // triangles it collides. Returns the closest triangle.
    // \param ignoreInvisible - if true, invisble objects are ignored
    bool castRay(
            const Eigen::Vector3d& origin,
            const Eigen::Vector3d& normal,
            SGLeafNode** leafNodeOut,
            std::shared_ptr<AbstractPolygon>& polyOut,
            size_t& triangleIdOut,
            Eigen::Vector3d& intersectionPointOut,
            bool ignoreInvisible = true);

    // SG Insertion Methods
        // Imports the file at the specified path and stores a new
        // node to the provided children node.
        SGNode* importFileAsChild(
                File file,
                SGChildrenNode* parent,
                bool renderOnlyOuterFaces = true);

        // If a geometry is represented by multiple geometry files like in the
        // (.node, .face) or (.node, .ele) file format, then this method
        // should be used.
        SGNode* importFilesAsChild(
                const std::vector<File>& files,
                SGChildrenNode* parent);

        // Combines and exports each geometry of the given nodes subgraph. If
        // node is a leaf, then the leafs geometry is exported.
        // Supported formats are:
        // -> .obj
        // -> (.node, .ele) for Polygon2D or (.node, .face) for Polygon3D
        //    (The exported file must be specified as .node, the corresponding
        //     .ele or .face file will automatically be inserted in the same
        //     folder).
        void exportToSingleFile(const File& file, SGNode* node);

        // Exports each geometry of the given nodes subgraph as single file in
        // the given format.
        // Supported formats are:
        // -> .obj
        // -> (.node, .face) for Polygon2D or (.node, .ele) for Polygon3D
        // \param folder - the target folder in which the files should be stored.
        void exportToMultipleFiles(const File& folder,
                                   const std::string& format,
                                   SGNode* node);

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

        // Converts the polygon of the given leaf node. Overwrites the old
        // geometry.
        // Returns the given leaf node.
        SGLeafNode* create3DGeometryFrom2D(
                SGLeafNode* leafNode,
                const MeshCriteria& meshCriteria,
                bool renderOnlyOuterFaces = true);

        // Converts the polygon of the given leafNode and sets it as the
        // geometry of a new node that is added to the given parent node.
        SGLeafNode* create3DGeometryFrom2D(
                std::string name,
                SGChildrenNode* parent,
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
        // Creates a FEMObject from the AbstractPolygon stored in the given leaf data.
        // If there is already a simulation object, it is removed beforehand.
        // In that case collision state and masses are reused.
        std::shared_ptr<FEMObject> createFEMObject(
                const std::shared_ptr<SceneLeafData>& ld,
                double mass = 1.0);

        // Creates a RigidBody from the AbstractPolygon stored in the given leaf data.
        // If there is already a simulation object, it is removed beforehand.
        // In that case collision state and masses are reused.
        std::shared_ptr<RigidBody> createRigidBody(
                const std::shared_ptr<SceneLeafData>& ld,
                double mass,
                bool iStatic = false);

        // This method is deprecated.
        // Creates a sphere based collision object.
        void createCollidable(
                const std::shared_ptr<SceneLeafData>& ld,
                double collisionSphereRadiusFactor = 0.1);

        // Creates a collidable object for the given leaf data and adds it
        // to the collision system. Requires a simulation object to be added
        // beforehand, else it does nothing.
        void createCollidable(
                const std::shared_ptr<SceneLeafData>& ld,
                const std::shared_ptr<MeshInterpolatorFEM>& interpolation);

        // Removes the simulation object from the simulation and sets the
        // reference in the scene node null.
        void removeSimulationObject(
                const std::shared_ptr<SceneLeafData>& ld);

        // Searches the scene graph for the scene node with the given
        // simulation object.
        // Removes the simulation object from the simulation and sets the
        // reference in the scene node null.
        void removeSimulationObject(
                const std::shared_ptr<SimulationObject>& so);


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
                GeometricData* geo,
                double mass = 1.0);

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
                std::shared_ptr<AbstractPolygon> polygon,
                Eigen::Vector position,
                bool renderOnlyOuterFaces = true);

        SGLeafNode* createLeafNode(
                std::string name,
                SGChildrenNode* parent,
                std::shared_ptr<AbstractPolygon> polygon,
                Eigen::Affine3d transform = Eigen::Affine3d::Identity(),
                bool renderOnlyOuterFaces = true);

    // Getters
        SGSceneGraph* getSceneGraph();

        SGNode* getSceneNodeByName(std::string name);

        // Searches the scene graph and returns the first found scene node that
        // has the given simulation object. If there is none, it returns nullptr.
        SGLeafNode* getSceneNodeBySimulationObject(SimulationObject* so);

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
