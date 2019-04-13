#include "SGControl.h"
#include "SGTraverserFactory.h"

#include <ApplicationControl.h>
#include <io/MeshIO.h>
#include <scene/data/geometric/GeometricDataFactory.h>
#include <scene/data/geometric/GeometricPoint.h>
#include <scene/data/geometric/Polygon2D.h>
#include <scene/data/geometric/Polygon3D.h>

#include <scene/model/ModelFactory.h>
#include <scene/model/PolygonRenderModel.h>
#include <scene/model/PolygonRenderModelImproved.h>

#include <simulation/fem/FEMObject.h>
#include <simulation/fem/SimulationPoint.h>

#include <scene/VertexCollection.h>
#include <scene/data/GeometricDataVisitor.h>
#include <scene/data/simulation/FEMData.h>
#include <simulation/SimulationObjectFactory.h>
#include <simulation/fem/FEMSimulation.h>
#include <simulation/forces/LinearForce.h>
#include <simulation/models/RigidRenderModel.h>
#include <simulation/rigid/RigidBody.h>
#include <ui/UIControl.h>
#include <ui/selection/SelectionControl.h>
#include <ui/selection/SelectionVertices.h>

#include <memory>

SGControl::SGControl()
{
    mSceneGraph = new SGSceneGraph();
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
//    mAc->getSimulationControl()->clearSimulationObjects();
//    mSceneGraph->getRoot()->clear();
}

SGLeafNode* SGControl::importFileAsChild(
        std::string path,
        std::string name,
        SGChildrenNode* parent,
        bool renderOnlyOuterFaces)
{
    // Geometric Data
    std::shared_ptr<Polygon2D> poly2 = std::shared_ptr<Polygon2D>(
                MeshIO::instance()->loadOff(path.c_str()));
    return createLeafNode(name, parent, poly2, renderOnlyOuterFaces);
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
    poly2->translate(position);

    return createLeafNode(name, parent, poly2, renderOnlyOuterFaces);
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
    poly2->translate(position);

    return createLeafNode(name, parent, poly2, renderOnlyOuterFaces);
}

SGLeafNode* SGControl::create3DGeometryFrom2D(
        SGLeafNode* leafNode,
        double cellSize,
        double cellRadiusEdgeRatio,
        bool renderOnlyOuterFaces)
{

    class Geo2DTo3DVisitor : public GeometricDataVisitor
    {
    public:
        Geo2DTo3DVisitor(double _cellSize, double _cellRadiusEdgeRatio)
            : cellSize(_cellSize)
            , cellRadiusEdgeRatio(_cellRadiusEdgeRatio)
        {

        }

        virtual void visit(Polygon2D& poly2)
        {
            returnValue = std::make_shared<Polygon3D>(
                        GeometricDataFactory::createPolygon3DFromPolygon2D(
                            poly2, cellSize, cellRadiusEdgeRatio));
        }

        virtual void visit(Polygon3D& /*polygon3D*/)
        {
            std::cout << "Geometric data is already a Polygon3D\n";
        }

        virtual void visit(GeometricPoint& /*point*/)
        {
            std::cout << "Can not convert a GeometricPoint to a Polyon3D\n";
        }

        double cellSize;
        double cellRadiusEdgeRatio;
        std::shared_ptr<Polygon3D> returnValue;
    };

    class Poly2To3Visitor : public SGNodeVisitor
    {
    public:
        Poly2To3Visitor(double _cellSize, double _cellRadiusEdgeRatio)
            : cellSize(_cellSize)
            , cellRadiusEdgeRatio(_cellRadiusEdgeRatio)
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
            Geo2DTo3DVisitor v(cellSize, cellRadiusEdgeRatio);
            leafNode->getData()->getGeometricData()->accept(v);
            returnValue = v.returnValue;
        }

        double cellSize;
        double cellRadiusEdgeRatio;
        std::shared_ptr<Polygon3D> returnValue;
    } v(cellSize, cellRadiusEdgeRatio);

    leafNode->accept(v);
    std::shared_ptr<Polygon3D> poly3 = v.returnValue;

    if (poly3)
    {
        std::shared_ptr<SceneLeafData> leafData = leafNode->getData();
        leafData->setGeometricData(poly3);

        // Render Model
        std::shared_ptr<PolygonRenderModelImproved> renderModel =
                ModelFactory::createPolygonRenderModelImproved(
                    mAc->getRenderModelManager(), poly3, renderOnlyOuterFaces);
        leafData->setRenderModel(renderModel);
    }

    return leafNode;
}

SGLeafNode*SGControl::createSimulationPoint(
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

void SGControl::createFEMObject(const std::shared_ptr<SceneLeafData>& ld)
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
            std::shared_ptr<FEMObject> femObj =
                    std::shared_ptr<FEMObject>(
                    SimulationObjectFactory::createFEMObject(
                        sgc.mAc->getSimulationControl()->getDomain(),
                            std::static_pointer_cast<Polygon3D>(polygon3D.shared_from_this())));
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
    } gdVisitor(*this, ld);

    ld->getGeometricData()->accept(gdVisitor);
}

void SGControl::createRigidBody(const std::shared_ptr<SceneLeafData>& ld, double mass, bool isStatic)
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

            std::shared_ptr<SimulationObject> so = ld->getSimulationObject();
            if (so)
            {
                // There is already a simulation object.
                // Remove that first from the simulation.
                sgc.mAc->getSimulationControl()->removeSimulationObject(so);
            }
            std::shared_ptr<RigidBody> rigid =
                    std::shared_ptr<RigidBody>(
                    SimulationObjectFactory::createRigidBody(
                        sgc.mAc->getSimulationControl()->getDomain(),
                            std::static_pointer_cast<Polygon3D>(poly.shared_from_this()), mass));
            rigid->setStatic(isStatic);
            sgc.mAc->getSimulationControl()->addSimulationObject(rigid);

//            sgc.mAc->getSimulationControl
            //TOD:  change render model

            // TODO:
            // simplify simulation object handling
            ld->setSimulationObject(rigid);
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

    } gdVisitor(*this, ld, mass, isStatic);


    ld->getGeometricData()->accept(gdVisitor);
}

void SGControl::createCollidable(const std::shared_ptr<SceneLeafData>& ld)
{
    std::shared_ptr<SimulationObject> so = ld->getSimulationObject();
    if (so)
    {
        mAc->getSimulationControl()->addCollisionObject(so);
    }
    else
    {
        std::cout << "There is no SimulationObject from which a CollisionObject could be created\n";
    }
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
        leafNode->getData()->setSimulationObject(
                    std::shared_ptr<SimulationObject>(createCorrespondingSimulationObject(geo)));
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
        void visitImpl(SGNode* node)
        {
            // update SGTree
            SGChildrenNode* parentNode = node->getParent();
            parentNode->removeChild(node); // TODO: why do these nodes have no tree listening to them?

            // update QTreeWidget
//            QTreeWidgetItem* parentItem = item->parent();
//            parentItem->removeChild(item);

            // update UISGControl
            //uiControl.mUISGControl->removeNode(item);
        }

        virtual void visit(SGChildrenNode* childrenNode)
        {
            visitImpl(childrenNode);
        }

        virtual void visit(SGLeafNode* leafNode)
        {
            // remove simulation object
            // TODO: here are some memory leaks because it is not save yet
            // to remove simulation object due to the simulation thread first
            // having to remove the simulation obejct first but it only does
            // that after each simulation step.
            // Also the geometry can not be removed because it contains the
            // position vector that is adapted by the simulation object.
            std::shared_ptr<SimulationObject> so = leafNode->getData()->getSimulationObject();
            if (so)
            {
                control.mAc->getSimulationControl()->removeSimulationObject(so);
            }

            // remode render model
            std::shared_ptr<RenderModel> rm = leafNode->getData()->getRenderModel();
            if (rm)
            {
                rm->removeFromRenderer(control.mAc->getUIControl()->getRenderer());
            }

            // remove geometric data
            //GeometricData* gd = leafNode->getData()->getGeometricData();
//            if (gd)
//            {
//                //delete gd;
//            }
            // TODO: how to safely remove SimulatoinObject
            //      RenderModel and GeometricData

            VertexCollection* vc =
                    control.mAc->getUIControl()->getSelectionControl()->getSelectionVertices()
                    ->getSelectedVertexCollection();
            vc->removeVertices(leafNode->getData());

            // remove node in scene graph
            visitImpl(leafNode);

            // this causes a segmentation fault because the mPositoins memory of polygon3d is
            // freed while the simulaion is accessing it
            // and because it removes the simulation object itself...
//            delete leafNode->getData();

        }

        SGControl& control;
    } visitor(*this);

    // item is parent item
    node->accept(visitor);
}

SGSceneGraph* SGControl::getSceneGraph()
{
    return mSceneGraph;
}

SGNode* SGControl::getSceneNodeByName(std::string name)
{
    return mSceneGraph->getRoot()->searchNodeByName(name);
}

SGLeafNode* SGControl::createLeafNode(
        std::string name,
        SGChildrenNode* parent,
        std::shared_ptr<Polygon> polygon,
        bool renderOnlyOuterFaces)
{
    SGLeafNode* leafNode = new SGLeafNode(name);
    std::shared_ptr<SceneLeafData> leafData = std::make_shared<SceneLeafData>(leafNode);

    leafData->setGeometricData(polygon);

    // Render Model
    std::shared_ptr<PolygonRenderModelImproved> renderModel =
            ModelFactory::createPolygonRenderModelImproved(
                mAc->getRenderModelManager(),
                polygon,
                renderOnlyOuterFaces);

    leafData->setRenderModel(renderModel);

    // Create leaf node and add to scene graph
    leafNode->setData(leafData);
    parent->addChild(leafNode);

    return leafNode;
}
