#include "SGControl.h"
#include "SGTraverserFactory.h"

#include <ApplicationControl.h>
#include <io/MeshIO.h>
#include <scene/data/geometric/GeometricDataFactory.h>
#include <scene/data/geometric/GeometricPoint.h>
#include <scene/data/geometric/MeshInterpolationManager.h>
#include <scene/data/geometric/Polygon2D.h>
#include <scene/data/geometric/Polygon3D.h>

#include <scene/model/ModelFactory.h>
#include <scene/model/PolygonRenderModel.h>

#include <simulation/fem/FEMObject.h>
#include <simulation/fem/SimulationPoint.h>

#include <io/importers/OBJImporter.h>
#include <io/importers/TetImporter.h>
#include <modules/mesh_converter/MeshCriteria.h>
#include <scene/data/GeometricDataVisitor.h>
#include <scene/data/simulation/FEMData.h>
#include <simulation/SimulationObjectFactory.h>
#include <simulation/fem/FEMSimulation.h>
#include <simulation/forces/LinearForce.h>
#include <simulation/rigid/RigidBody.h>
#include <ui/UIControl.h>
#include <ui/selection/SelectionControl.h>
#include <ui/selection/SelectionVertices.h>

#include <memory>

SGControl::SGControl()
{
    mSceneGraph = new SGSceneGraph();
    mSceneGraph->getRoot()->setData(std::make_shared<SceneData>(
                                        mSceneGraph->getRoot()));
    mSceneGraph->addTreeListener(this);
}

void SGControl::initialize(ApplicationControl* ac)
{
    mAc = ac;
}

void SGControl::clearScene()
{
//    mAc->getSimulationControl()>cl
    std::vector<SGNode*> children = mSceneGraph->getRoot()->getChildren();
    for (SGNode* node : children)
    {
        removeNode(node);
    }
    mAc->getMeshInterpolationManager()->clearInterpolators();
//    mAc->getSimulationControl()->clearSimulationObjects();
//    mSceneGraph->getRoot()->clear();
}

SGNode* SGControl::importFileAsChild(
        File file,
        SGChildrenNode* parent,
        bool renderOnlyOuterFaces)
{
    std::shared_ptr<Polygon2D> poly2;
    std::string path = file.getPath();
    std::string extension = file.getExtension();
    if (extension == ".off")
    {
        poly2 = std::shared_ptr<Polygon2D>(
                    MeshIO::instance()->loadOff(path.c_str()));

        return createLeafNode(file.getName(), parent, poly2,
                              Eigen::Vector::Zero(), renderOnlyOuterFaces);
    }
    else if (extension == ".obj")
    {
        OBJImporter importer;
        SGNode* node = importer.importFile(file, mAc);
        parent->addChild(node);
        return node;
    }
    else if (extension == ".tet")
    {
        TetImporter importer;
        SGNode* node = importer.importFile(file, mAc);
        parent->addChild(node);
        return node;
    }

    std::cout << "Filetype " << extension << " not supported.\n";

    return nullptr;
}

SGLeafNode* SGControl::createBox(
        std::string name,
        SGChildrenNode* parent,
        Vector position,
        double width,
        double height,
        double length,
        bool renderOnlyOuterFaces)
{
    // Create box and translate to target position
    std::shared_ptr<Polygon2D> poly2 =
            std::make_shared<Polygon2D>(GeometricDataFactory::create2DBox(width, length, height));

    return createLeafNode(name, parent, poly2, position, renderOnlyOuterFaces);
}

SGLeafNode* SGControl::createSphere(
        std::string name,
        SGChildrenNode* parent,
        Vector position,
        double radius,
        int resolution,
        bool renderOnlyOuterFaces)
{
    // Create sphere and translate to target position
    std::shared_ptr<Polygon2D> poly2 =
            std::make_shared<Polygon2D>(
                GeometricDataFactory::create2DSphere(radius, resolution));

    return createLeafNode(name, parent, poly2, position, renderOnlyOuterFaces);
}

SGLeafNode* SGControl::create3DGeometryFrom2D(
        SGLeafNode* leafNode,
        const MeshCriteria& meshCriteria,
        bool renderOnlyOuterFaces)
{
    class Geo2DTo3DVisitor : public GeometricDataVisitor
    {
    public:
        Geo2DTo3DVisitor(const MeshCriteria& _meshCriteria)
            : meshCriteria(_meshCriteria)
        {

        }

        virtual void visit(Polygon2D& poly2)
        {
            returnValue = std::make_shared<Polygon3D>(
                        GeometricDataFactory::createPolygon3DFromPolygon2D(
                            poly2,
                            meshCriteria));
        }

        virtual void visit(Polygon3D& /*polygon3D*/)
        {
            std::cout << "Geometric data is already a Polygon3D\n";
        }

        virtual void visit(GeometricPoint& /*point*/)
        {
            std::cout << "Can not convert a GeometricPoint to a Polyon3D\n";
        }

        const MeshCriteria& meshCriteria;
        std::shared_ptr<Polygon3D> returnValue;
    };

    class Poly2To3Visitor : public SGNodeVisitor
    {
    public:
        Poly2To3Visitor(const MeshCriteria& _meshCriteria)
            : meshCriteria(_meshCriteria)
        {

        }

        virtual void visit(SGChildrenNode* /*childrenNode*/)
        {
            // children nodes have no leaf node
            std::cout << "Can not transform to 2D because children nodes have no "
                         "geometric data.\n";
        }

        virtual void visit(SGLeafNode* leafNode)
        {
            Geo2DTo3DVisitor v(meshCriteria);
            leafNode->getData()->getGeometricData()->accept(v);
            returnValue = v.returnValue;
        }

        const MeshCriteria& meshCriteria;
        std::shared_ptr<Polygon3D> returnValue;
    } v(meshCriteria);

    leafNode->accept(v);
    std::shared_ptr<Polygon3D> poly3 = v.returnValue;

    if (poly3)
    {
        std::shared_ptr<SceneLeafData> leafData = leafNode->getData();
        leafData->setGeometricData(poly3);

        // Render Model
        std::shared_ptr<PolygonRenderModel> renderModel =
                ModelFactory::createPolygonRenderModelImproved(
                    mAc->getRenderModelManager(), poly3, renderOnlyOuterFaces);
        leafData->setRenderModel(renderModel);
    }

    return leafNode;
}

SGLeafNode* SGControl::createSimulationPoint(
        std::string name,
        SGChildrenNode* parent,
        Vector position)
{
    SGLeafNode* leafNode = SGTreeNodeFactory::createLeafNode(parent, name);
    leafNode->setData(std::make_shared<SceneLeafData>(leafNode));
    leafNode->getData()->setGeometricData(
                std::make_shared<GeometricPoint>(position));
    createAndSetCorrespondingSimulationObject(leafNode);
    return leafNode;
}

void SGControl::createLinearForce(
        std::string name,
        SGChildrenNode* parent,
        SimulationPointRef source,
        const Vector& target,
        double strength)
{
    // add linear force
    SGLeafNode* leafNode = createSimulationPoint(name, parent, target);

    // create the linear force
    std::shared_ptr<LinearForce> lf = std::make_shared<LinearForce>(
                source,
                SimulationPointRef(leafNode->getData()->getSimulationObjectRaw(), 0),
                strength);
    mAc->getSimulationControl()->addLinearForce(lf);
}

std::shared_ptr<FEMObject> SGControl::createFEMObject(
        const std::shared_ptr<SceneLeafData>& ld)
{
    // create FEM object if possible
    // check in list of
    class GDVisitor : public GeometricDataVisitor
    {
    public:
        GDVisitor(SGControl& _sgc, const std::shared_ptr<SceneLeafData>& _ld)
            : sgc(_sgc)
            , ld(_ld)
        {

        }

        virtual void visit(Polygon2D& /*polygon2D*/)
        {
            std::cout << "Can not create FEM object from Polygon2D." << std::endl;
        }

        virtual void visit(Polygon3D& polygon3D)
        {
            std::shared_ptr<SimulationObject> so = ld->getSimulationObject();
            if (so)
            {
                // There is already a simulation object.
                // Remove that first from the simulation.
                sgc.mAc->getSimulationControl()->removeSimulationObject(so);
            }
            femObj = std::shared_ptr<FEMObject>(
                        SimulationObjectFactory::createFEMObject(
                            sgc.mAc->getSimulationControl()->getDomain(),
                            std::static_pointer_cast<Polygon3D>(
                                polygon3D.shared_from_this())));
            sgc.mAc->getSimulationControl()->addSimulationObject(femObj);

            polygon3D.changeRepresentationToWS();

            // TODO:
            // simplify simulation object handling
            ld->setSimulationObject(femObj);
            ld->getRenderModelRaw()->revalidate();

        }

        virtual void visit(GeometricPoint& /*point*/)
        {
            std::cout << "Can not create FEM object from point." << std::endl;
        }
        SGControl& sgc;
        std::shared_ptr<SceneLeafData> ld;
        std::shared_ptr<FEMObject> femObj;
    } gdVisitor(*this, ld);

    ld->getGeometricData()->accept(gdVisitor);
    return gdVisitor.femObj;
}

std::shared_ptr<RigidBody> SGControl::createRigidBody(
        const std::shared_ptr<SceneLeafData>& ld, double mass, bool isStatic)
{
    // create FEM object if possible
    // check in list of
    class GDVisitor : public GeometricDataVisitor
    {
    public:
        GDVisitor(SGControl& _sgc, const std::shared_ptr<SceneLeafData>& _ld, double _mass, bool _isStatic)
            : sgc(_sgc)
            , ld(_ld)
            , mass(_mass)
            , isStatic(_isStatic)
        {

        }

        void visitPoly(Polygon& poly)
        {
            Vector center = poly.calculateCenterVertex();
            poly.changeRepresentationToBS(center);

            rb = std::shared_ptr<RigidBody>(
                        SimulationObjectFactory::createRigidBody(
                            sgc.mAc->getSimulationControl()->getDomain(),
                            std::static_pointer_cast<Polygon3D>(poly.shared_from_this()),
                            mass));
            rb->setStatic(isStatic);

            // remove the old simulation object
            std::shared_ptr<SimulationObject> so = ld->getSimulationObject();
            if (so)
            {
                // There is already a simulation object.
                // Remove that first from the simulation.
                sgc.mAc->getSimulationControl()->removeSimulationObject(so);
            }

            // add the new simulation object
            sgc.mAc->getSimulationControl()->addSimulationObject(rb);

//            sgc.mAc->getSimulationControl
            //TOD:  change render model

            // TODO:
            // simplify simulation object handling
            ld->setSimulationObject(rb);
            ld->getRenderModelRaw()->revalidate();
        }

        virtual void visit(Polygon2D& poly2)
        {
            visitPoly(poly2);
        }

        virtual void visit(Polygon3D& poly3)
        {
            visitPoly(poly3);
        }

        virtual void visit(GeometricPoint& /*point*/)
        {
            std::cout << "Can not create Rigid object from point." << std::endl;
        }

        SGControl& sgc;
        const std::shared_ptr<SceneLeafData>& ld;
        double mass;
        bool isStatic;
        std::shared_ptr<RigidBody> rb;

    } gdVisitor(*this, ld, mass, isStatic);

    ld->getGeometricData()->accept(gdVisitor);
    return gdVisitor.rb;
}

void SGControl::createCollidable(const std::shared_ptr<SceneLeafData>& ld,
                                 double collisionSphereRadiusFactor)
{
    std::shared_ptr<SimulationObject> so = ld->getSimulationObject();
    if (so)
    {
        mAc->getSimulationControl()->addCollisionObject(
                    so, collisionSphereRadiusFactor);
    }
    else
    {
        std::cout << "There is no SimulationObject from which a CollisionObject could be created\n";
    }
}

void SGControl::removeSimulationObject(const std::shared_ptr<SceneLeafData>& ld)
{
    std::shared_ptr<SimulationObject> so = ld->getSimulationObject();
    if (so)
    {
        mAc->getSimulationControl()->removeSimulationObject(so);
        ld->setSimulationObject(nullptr);
    }
    else
    {
        std::cout << "Cannot remove simulation object from leaf data because"
                     "there is none.\n";
    }
}

void SGControl::removeSimulationObject(const std::shared_ptr<SimulationObject>& so)
{
    SGLeafNode* leafNode =
            mAc->getSGControl()->getSceneNodeBySimulationObject(so.get());
    if (leafNode)
    {
        removeSimulationObject(leafNode->getData());
    }
    else
    {
        std::cout << "Can not remove scene node with simulation object "
                     "because it is not part of the scene graph.\n";
    }
}

SGChildrenNode* SGControl::createChildrenNode(SGChildrenNode* parent, std::string name)
{
    return SGTreeNodeFactory::createChildrenNode(parent, name);
}

SGLeafNode* SGControl::createAndAddLeafNodeToRoot(std::string name)
{
    // find node with the name
    SGLeafNode* leafNode =
            SGTreeNodeFactory::createLeafNode(mSceneGraph->getRoot(), name);
    return leafNode;
}

SGLeafNode* SGControl::createAndAddLeafNode(std::string parentName, std::string childName)
{
    SGNode* node = getSceneNodeByName(parentName);

    // if no node by that name is found, return nullptr
    if (node == nullptr)
        return nullptr;

    // if the found node can hold children (aka it is a ChildrenNode)
    // then it gets added a SGLeafNode, else return null
    SGLeafNode* leafNode = nullptr;
    class Visitor : public SGNodeVisitor
    {
    public:
        Visitor(std::string& _parentName,
                std::string& _childName,
                SGLeafNode* _leafNode)
            : parentName(_parentName)
            , childName(_childName)
            , leafNode(_leafNode)
        {

        }

        virtual void visit(SGChildrenNode* childrenNode)
        {
            leafNode =
                    SGTreeNodeFactory::createLeafNode(
                        childrenNode,
                        childName);
        }

        virtual void visit(SGLeafNode* /*leafNode*/)
        {
            leafNode = nullptr;
        }

        std::string& parentName;
        std::string& childName;
        SGLeafNode* leafNode;
    } v(parentName, childName, leafNode);

    node->accept(v);
    return leafNode;
}

void SGControl::createAndSetCorrespondingSimulationObject(SGLeafNode* leafNode)
{
    //TODO is this method only working for simulation points, not fem objects?
    if (GeometricData* geo = leafNode->getData()->getGeometricDataRaw())
    {
        std::shared_ptr<SimulationObject> so =
                std::shared_ptr<SimulationObject>(
                    createCorrespondingSimulationObject(geo));
        leafNode->getData()->setSimulationObject(so);
        mAc->getSimulationControl()->addSimulationObject(so);
    }
}

SimulationObject* SGControl::createCorrespondingSimulationObject(GeometricData* geo)
{
    class CorrespondingSimulationObjectCreator : public GeometricDataVisitor
    {
    public:
        CorrespondingSimulationObjectCreator(SGControl& _sgc)
            : sgc(_sgc)
        {
        }

        virtual void visit(Polygon2D& poly2)
        {
            target = SimulationObjectFactory::createRigidBody(
                        sgc.mAc->getSimulationControl()->getDomain(),
                        std::static_pointer_cast<Polygon3D>(poly2.shared_from_this()), 1.0);
        }

        virtual void visit(Polygon3D& poly3)
        {
            target = SimulationObjectFactory::createFEMObject(
                        sgc.mAc->getSimulationControl()->getDomain(),
                        std::static_pointer_cast<Polygon3D>(poly3.shared_from_this()));
        }

        virtual void visit(GeometricPoint& point)
        {
            target = SimulationObjectFactory::createSimulationPoint(
                        sgc.mAc->getSimulationControl()->getDomain(),
                        std::static_pointer_cast<GeometricPoint>(
                            point.shared_from_this()));
        }

        SGControl& sgc;
        SimulationObject* target;
    } visitor(*this);

    geo->accept(visitor);
    return visitor.target;
}

SGTraverser SGControl::createSceneGraphTraverser()
{
    return SGTraverserFactory::createDefaultSGTraverser(
                mSceneGraph->getRoot());
}

void SGControl::removeNode(SGNode* node)
{
    // create new node and add it to parent
    class RemoveChildrenNodeVisitor : public SGNodeVisitor
    {
    public:
        RemoveChildrenNodeVisitor(SGControl& _control)
            : control(_control)
        {

        }

        virtual void visit(SGChildrenNode* /*childrenNode*/)
        {
        }

        virtual void visit(SGLeafNode* leafNode)
        {
            // remove simulation object
            std::shared_ptr<SimulationObject> so = leafNode->getData()->getSimulationObject();
            if (so)
            {
                control.mAc->getSimulationControl()->removeSimulationObject(so);
            }

            // remode render model
            std::shared_ptr<RenderModel> rm = leafNode->getData()->getRenderModel();
            if (rm)
            {
                // TODO: why do these nodes have no tree listening to them?
                rm->removeFromRenderer(control.mAc->getUIControl()->getRenderer());
            }

            SelectionVertices* sv = control.mAc->getUIControl()->
                    getSelectionControl()->getSelectionVertices();
            sv->removeVertices(leafNode->getData());

            std::shared_ptr<GeometricData> gd =
                    leafNode->getData()->getGeometricData();
            if (gd->getType() == GeometricData::Type::POLYGON)
            {
                std::shared_ptr<Polygon> poly =
                        std::static_pointer_cast<Polygon>(gd);
                control.mAc->getMeshInterpolationManager()->removeInterpolator(poly);
            }
        }

        SGControl& control;
    } visitor(*this);

    // item is parent item
    SGTraverser traverser = SGTraverserFactory::createDefaultSGTraverser(node);
    traverser.traverse(visitor);

    static_cast<SGChildrenNode*>(node)->getParent()->removeChild(node);
}

SGLeafNode* SGControl::createLeafNode(
        std::string name,
        SGChildrenNode* parent,
        std::shared_ptr<Polygon> polygon,
        Eigen::Vector position,
        bool renderOnlyOuterFaces)
{
    Eigen::Affine3d translation = Eigen::Affine3d::Identity() * Eigen::Translation3d(position);
    return createLeafNode(name, parent, polygon,
                          translation,
                          renderOnlyOuterFaces);
}

SGLeafNode* SGControl::createLeafNode(
        std::string name,
        SGChildrenNode* parent,
        std::shared_ptr<Polygon> polygon,
        Affine3d transform,
        bool renderOnlyOuterFaces)
{
    SGLeafNode* leafNode = new SGLeafNode(name);
    std::shared_ptr<SceneLeafData> leafData = std::make_shared<SceneLeafData>(leafNode);

    if (polygon)
    {
        polygon->transform(transform);
        leafData->setGeometricData(polygon);

        // Render Model
        std::shared_ptr<PolygonRenderModel> renderModel =
                ModelFactory::createPolygonRenderModelImproved(
                    mAc->getRenderModelManager(),
                    polygon,
                    renderOnlyOuterFaces);

        leafData->setRenderModel(renderModel);
    }


    // Create leaf node and add to scene graph
    leafNode->setData(leafData);
    parent->addChild(leafNode);

    return leafNode;
}

SGSceneGraph* SGControl::getSceneGraph()
{
    return mSceneGraph;
}

SGNode* SGControl::getSceneNodeByName(std::string name)
{
    return mSceneGraph->getRoot()->searchNodeByName(name);
}

SGLeafNode* SGControl::getSceneNodeBySimulationObject(SimulationObject* so)
{
    class RemoveChildrenNodeVisitor : public SGNodeVisitor
    {
    public:
        RemoveChildrenNodeVisitor(SimulationObject* _so, SGTraverser& _traverser)
            : so(_so)
            , traverser(_traverser)
        {
            leaf = nullptr;
        }

        virtual void visit(SGChildrenNode* /*childrenNode*/)
        {
        }

        virtual void visit(SGLeafNode* leafNode)
        {
            if (leafNode->getData()->getSimulationObjectRaw() == so)
            {
                leaf = leafNode;
                traverser.setEndSearchEarly(true);
            }
        }

        SGLeafNode* leaf;
        SimulationObject* so;
        SGTraverser& traverser;
    };

    SGTraverser traverser =
            SGTraverserFactory::createDefaultSGTraverser(mSceneGraph->getRoot());

    RemoveChildrenNodeVisitor v(so, traverser);
    traverser.traverse(v);

    return v.leaf;
}

void SGControl::notifyLeafDataChanged(SGNode* /*source*/,
                                      std::shared_ptr<SceneLeafData>& /*data*/)
{

}

void SGControl::notifyChildAdded(SGNode* /*source*/, SGNode* childNode)
{
    if (childNode->isLeaf())
    {
        SGLeafNode* leafNode = static_cast<SGLeafNode*>(childNode);
        if (!leafNode->getData())
        {
            leafNode->setData(std::make_shared<SceneLeafData>(leafNode));
        }
    }
    else
    {
        SGChildrenNode* childrenNode = static_cast<SGChildrenNode*>(childNode);
        if (!childrenNode->getData())
        {
            childrenNode->setData(std::make_shared<SceneData>(childrenNode));
        }
    }
}

void SGControl::notifyChildRemoved(SGNode* /*source*/, SGNode* /*childNode*/)
{

}

void SGControl::notifyChildrenDataChanged(SGNode* /*source*/,
                                          std::shared_ptr<SceneData>& /*data*/)
{

}

void SGControl::notifyParentChanged(SGNode* /*source*/, SGNode* /*parent*/)
{

}

void SGControl::notifyNameChanged(SGNode* /*source*/, std::string /*name*/)
{

}

void SGControl::notifyTreeChanged(SGNode* /*source*/,
                                  Tree<std::shared_ptr<SceneData>,
                                  std::shared_ptr<SceneLeafData> >* /*tree*/)
{

}
