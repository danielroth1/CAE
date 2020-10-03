#include "SGControl.h"
#include "SGTraverserFactory.h"

#include <ApplicationControl.h>
#include <io/MeshIO.h>
#include <scene/data/geometric/GeometricDataFactory.h>
#include <scene/data/geometric/GeometricPoint.h>
#include <scene/data/geometric/MeshInterpolationManager.h>
#include <scene/data/geometric/MeshInterpolatorFEM.h>
#include <scene/data/geometric/Polygon2D.h>
#include <scene/data/geometric/Polygon3D.h>

#include <scene/model/ModelFactory.h>
#include <scene/model/PolygonRenderModel.h>

#include <simulation/fem/FEMObject.h>
#include <simulation/fem/SimulationPoint.h>

#include <io/exporters/OBJExporter.h>
#include <io/importers/OBJImporter.h>
#include <io/importers/TetGenImporter.h>
#include <io/importers/TetImporter.h>
#include <modules/interpolator/InterpolatorModule.h>
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

#include <map>
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
    mAc->getInterpolatorModule()->clearInterpolators();
    std::vector<SGNode*> children = mSceneGraph->getRoot()->getChildren();
    for (SGNode* node : children)
    {
        removeNode(node);
    }
//    mAc->getSimulationControl()->clearSimulationObjects();
    //    mSceneGraph->getRoot()->clear();
}

bool SGControl::castRay(
        const Vector3d& origin,
        const Vector3d& normal,
        SGLeafNode** leafNodeOut,
        std::shared_ptr<AbstractPolygon>& polyOut,
        size_t& triangleIdOut,
        Vector3d& intersectionPointOut,
        bool ignoreInvisible)
{
    // Iterate over the whole graph and check if the ray connects with any
    // polygon.

    SGTraverser castRayTraverser(mSceneGraph->getRoot());
    class CastRayVisitor : public SGNodeVisitorImpl
    {
    public:
        CastRayVisitor(
                    const Eigen::Vector3d& _origin,
                    const Eigen::Vector3d& _normal,
                    SGLeafNode** _leafNodeOut,
                    std::shared_ptr<AbstractPolygon>& _polyOut,
                    size_t& _triangleIdOut,
                    Eigen::Vector3d& _intersectionPointOut)
            : origin(_origin)
            , normal(_normal)
            , leafNodeOut(_leafNodeOut)
            , polyOut(_polyOut)
            , triangleIdOut(_triangleIdOut)
            , intersectionPointOut(_intersectionPointOut)
        {
            intersects = false;
            distance = std::numeric_limits<double>::max();
        }

        virtual void visit(SGLeafNode* leafNode)
        {
            if (leafNode->getData()->getGeometricData()->getType() ==
                    GeometricData::Type::POLYGON)
            {
                std::shared_ptr<AbstractPolygon> poly =
                        std::static_pointer_cast<AbstractPolygon>(leafNode->getData()->getGeometricData());

                size_t triangleIdTemp;
                Eigen::Vector2d baryTemp;
                double distanceTemp;
                if (poly->castRay(origin, normal, triangleIdTemp, baryTemp, distanceTemp))
                {
                    intersects = true;
                    if (distance > 0 && distanceTemp < distance)
                    {
                        distance = distanceTemp;
                        polyOut = poly;
                        triangleIdOut = triangleIdTemp;
                        *leafNodeOut = leafNode;
                        Face f = poly->getTopology().getFacesIndices()[triangleIdOut];
                        intersectionPointOut =
                                Eigen::Vector3d(
                                    (1 - baryTemp(0) - baryTemp(1)) * poly->getPosition(f[0]) +
                                    baryTemp(0) * poly->getPosition(f[1]) +
                                    baryTemp(1) * poly->getPosition(f[2]));
                    }
                }
            }
        }

        bool intersects;
        double distance;

    private:
        const Eigen::Vector3d& origin;
        const Eigen::Vector3d& normal;
        SGLeafNode** leafNodeOut;
        std::shared_ptr<AbstractPolygon>& polyOut;
        size_t& triangleIdOut;
        Eigen::Vector3d& intersectionPointOut;

    } visitor(origin, normal, leafNodeOut, polyOut, triangleIdOut, intersectionPointOut);
    castRayTraverser.setFilter(SGTraverserFactory::createVisibilityFilter(ignoreInvisible));
    castRayTraverser.traverse(visitor);

    return visitor.intersects;
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

SGNode* SGControl::importFilesAsChild(
        const std::vector<File>& files,
        SGChildrenNode* parent)
{
    auto itNode = std::find_if(files.begin(), files.end(), [](const File& f)
    {
        return f.getExtension() == ".node";
    });
    auto itFace = std::find_if(files.begin(), files.end(), [](const File& f)
    {
        return f.getExtension() == ".face";
    });
    auto itEle = std::find_if(files.begin(), files.end(), [](const File& f)
    {
        return f.getExtension() == ".ele";
    });

    if (itNode != files.end() && itFace == files.end() && itEle == files.end())
    {
        std::cout << "Import failed, cause: Provided .node file but no .face nor .ele.";
    }
    else if (itNode == files.end() && itFace != files.end())
    {
        std::cout << "Import failed, cause: Provided .face file but no corresponding .node.";
    }
    else if (itNode == files.end() && itEle != files.end())
    {
        std::cout << "Import failed, cause: Provided .ele file but no corresponding .node.";
    }

    if ((itNode != files.end() && itFace != files.end()) || // (node, face) pair
            (itNode != files.end() && itEle != files.end())) // (node, ele) pair
    {
        TetGenImporter importer;
        SGNode* node = importer.importFiles(files, mAc);
        if (parent)
            parent->addChild(node);
        return node;
    }
    return nullptr;
}

void SGControl::exportToSingleFile(const File& file, SGNode* node)
{
    if (node->isLeaf())
    {
        std::shared_ptr<GeometricData> gd =
                static_cast<SGLeafNode*>(node)->getData()->getGeometricData();

        std::string path = file.getPath();
        std::string extension = file.getExtension();
        if (extension == ".obj")
        {
            OBJExporter exporter;
            exporter.exportToFile(file, gd);
        }
    }
    else
    {
        // combine all polygons to one and export this one or use an accessor
        // that accesses the data of multiple polygons.
    }
}

void SGControl::exportToMultipleFiles(
        const File& folder, const std::string& format, SGNode* node)
{
    // Iterate over the whole subgraph and store each found file in the folder.
    SGTraverser exporterTraverser(node);
    class ExporterVisitor : public SGNodeVisitorImpl
    {
    public:
        ExporterVisitor(const File& _folder, const std::string& _format)
            : folder(_folder)
            , format(_format)
        {

        }

        virtual void visit(SGLeafNode* leafNode)
        {
            File file(folder.getPath() + File::SEPARATOR + leafNode->getName() + format);
            auto it = filesMap.find(file.getPath());
            if (it != filesMap.end())
            {
                it->second++;
                std::string filePath =
                        file.getRelativePath() + File::SEPARATOR +
                        file.getName() + std::to_string(it->second) +
                        file.getExtension();
                files.push_back(filePath);
            }
            else
            {
                filesMap[file.getPath()] = 1;
                files.push_back(file.getPath());
            }
            leafs.push_back(leafNode);
        }

        std::vector<SGLeafNode*> leafs;
        std::vector<std::string> files;
        std::map<std::string, int> filesMap;
    private:
        const File& folder;
        const std::string& format;
    } visitor(folder, format);
    exporterTraverser.traverse(visitor);

    for (size_t i = 0; i < visitor.files.size(); ++i)
    {
        exportToSingleFile(visitor.files[i], visitor.leafs[i]);
    }
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

SGLeafNode* SGControl::create3DGeometryFrom2D(
        std::string name,
        SGChildrenNode* parent,
        SGLeafNode* leafNode,
        const MeshCriteria& meshCriteria,
        bool renderOnlyOuterFaces)
{
    std::shared_ptr<GeometricData> gd = leafNode->getData()->getGeometricData();
    SGLeafNode* newNode;
    if (gd != nullptr)
    {
        if (gd->getType() == GeometricData::Type::POLYGON)
        {
            std::shared_ptr<AbstractPolygon> poly = std::static_pointer_cast<AbstractPolygon>(gd);
            newNode = mAc->getSGControl()->createLeafNode(name, parent, poly);
            create3DGeometryFrom2D(newNode, meshCriteria, renderOnlyOuterFaces);
        }
    }
    return newNode;
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
        const std::shared_ptr<SceneLeafData>& ld, double mass)
{
    // create FEM object if possible
    // check in list of
    class GDVisitor : public GeometricDataVisitor
    {
    public:
        GDVisitor(SGControl& _sgc,
                  const std::shared_ptr<SceneLeafData>& _ld,
                  double _mass)
            : sgc(_sgc)
            , ld(_ld)
            , mass(_mass)
        {

        }

        virtual void visit(Polygon2D& /*polygon2D*/)
        {
            std::cout << "Can not create FEM object from Polygon2D." << std::endl;
        }

        virtual void visit(Polygon3D& polygon3D)
        {
            std::shared_ptr<SimulationObject> so = ld->getSimulationObject();
            bool colliding = sgc.mAc->getSimulationControl()->isCollidable(so);
            double massFinal = mass;
            if (so)
            {
                // There is already a simulation object.
                // Remove that first from the simulation.
                sgc.mAc->getSimulationControl()->getProxy()->removeSimulationObject(so);
                // Take the mass from the removed object if possible.
                massFinal = so->getMass();
            }
            polygon3D.changeRepresentationToWS();
            femObj = std::shared_ptr<FEMObject>(
                        SimulationObjectFactory::createFEMObject(
                            sgc.mAc->getSimulationControl()->getDomain(),
                            std::static_pointer_cast<Polygon3D>(polygon3D.shared_from_this()),
                            massFinal));
            sgc.mAc->getSimulationControl()->addSimulationObject(femObj);
            if (colliding)
            {
                sgc.mAc->getSimulationControl()->getProxy()->setCollidable(femObj, true);
            }

            // TODO: simplify simulation object handling
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
        double mass;
    } gdVisitor(*this, ld, mass);

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

        void visitPoly(AbstractPolygon& poly)
        {
            Vector center = poly.calculateCenterVertex();
            poly.changeRepresentationToBS(center);

            // remove the old simulation object
            std::shared_ptr<SimulationObject> so = ld->getSimulationObject();
            bool colliding = sgc.mAc->getSimulationControl()->isCollidable(so);
            double massFinal = mass;
            if (so)
            {
                // There is already a simulation object.
                // Remove that first from the simulation.
                sgc.mAc->getSimulationControl()->getProxy()->removeSimulationObject(so);
                // Take the mass from the removed object if possible.
                massFinal = so->getMass();
            }

            rb = std::shared_ptr<RigidBody>(
                        SimulationObjectFactory::createRigidBody(
                            sgc.mAc->getSimulationControl()->getDomain(),
                            std::static_pointer_cast<Polygon3D>(poly.shared_from_this()),
                            massFinal));
            rb->setStatic(isStatic);

            // add the new simulation object
            sgc.mAc->getSimulationControl()->addSimulationObject(rb);

            if (colliding)
            {
                sgc.mAc->getSimulationControl()->getProxy()->setCollidable(rb, true);
            }

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
        mAc->getSimulationControl()->getProxy()->addCollisionObject(
                    so, nullptr, collisionSphereRadiusFactor);
    }
    else
    {
        std::cout << "There is no SimulationObject from which a CollisionObject could be created.\n";
    }
}

void SGControl::createCollidable(
        const std::shared_ptr<SceneLeafData>& ld,
        const std::shared_ptr<MeshInterpolatorFEM>& interpolation)
{
    std::shared_ptr<SimulationObject> so = ld->getSimulationObject();
    if (so)
    {
        mAc->getSimulationControl()->getProxy()->setCollidable(interpolation->getTarget(), true);
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
        mAc->getSimulationControl()->getProxy()->removeSimulationObject(so);
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

SimulationObject* SGControl::createCorrespondingSimulationObject(
        GeometricData* geo, double mass)
{
    class CorrespondingSimulationObjectCreator : public GeometricDataVisitor
    {
    public:
        CorrespondingSimulationObjectCreator(SGControl& _sgc, double _mass)
            : sgc(_sgc)
            , mass(_mass)
        {
        }

        virtual void visit(Polygon2D& poly2)
        {
            target = SimulationObjectFactory::createRigidBody(
                        sgc.mAc->getSimulationControl()->getDomain(),
                        std::static_pointer_cast<Polygon3D>(poly2.shared_from_this()),
                        mass);
        }

        virtual void visit(Polygon3D& poly3)
        {
            target = SimulationObjectFactory::createFEMObject(
                        sgc.mAc->getSimulationControl()->getDomain(),
                        std::static_pointer_cast<Polygon3D>(poly3.shared_from_this()),
                        mass);
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
        double mass;
    } visitor(*this, mass);

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
    // Only true for the root node which can't be removed.
    if (node->getParent() == nullptr)
    {
        std::cout << "Can not remove root node.\n";
        return;
    }

    // Correctly removes all leaf nodes of the sub graph.
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
                control.mAc->getSimulationControl()->getProxy()->removeSimulationObject(so);
            }

            // remove render model
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
                std::shared_ptr<AbstractPolygon> poly =
                        std::static_pointer_cast<AbstractPolygon>(gd);
                control.mAc->getInterpolatorModule()->removeInterpolator(leafNode);
            }
        }

        SGControl& control;
    } visitor(*this);

    // item is parent item
    SGTraverser traverser = SGTraverserFactory::createDefaultSGTraverser(node);
    traverser.traverse(visitor);

    node->getParent()->removeChild(node);
}

SGLeafNode* SGControl::createLeafNode(
        std::string name,
        SGChildrenNode* parent,
        std::shared_ptr<AbstractPolygon> polygon,
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
        std::shared_ptr<AbstractPolygon> polygon,
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
    if (parent)
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
