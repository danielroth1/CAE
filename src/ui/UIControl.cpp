#include "KeyManager.h"
#include "ModulesUIControl.h"
#include "UIControl.h"

#include <GL/glew.h>
#include <ApplicationControl.h>
#include "scene/VertexGroup.h"
#include "main_window.h"
#include "SimulationControl.h"
#include <glwidget.h>
#include <QGuiApplication>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QListWidgetItem>
#include <qtreewidget.h>
#include <QLayout>
#include "rendering/Renderer.h"
#include <scene/data/SimulationData.h>
#include <scene/data/simulation/FEMData.h>
#include <scene/scene_graph/SGCore.h>
#include <scene/scene_graph/SGControl.h>
#include <scene/scene_graph/SGNodeVisitorFactory.h>
#include <scene/scene_graph/SGTraverserFactory.h>
#include <scene/scene_graph/SGControl.h>
#include <simulation/Simulation.h>
#include <simulation/fem/FEMSimulation.h>
#include <ui/qt/FileDialog.h>
#include <scene/data/geometric/GeometricDataFactory.h>
#include <scene/data/geometric/GeometricPoint.h>
#include <scene/data/geometric/AbstractPolygon.h>
#include <simulation/models/LinearForceRenderModel.h>
#include <ui/selection/Selection.h>
#include <ui/selection/SelectionControl.h>
#include <ui/selection/SelectionRectangle.h>
#include <ui/selection/SelectionRectangleModel.h>
#include <ui/selection/SelectionSceneData.h>
#include <ui/selection/SelectionSceneDataModel.h>
#include <ui/selection/SelectionVertices.h>
#include <ui/selection/SelectionVerticesModel.h>
#include <ui/scene_graph/SGQtWidgetManager.h>
#include <ui/scene_graph/SGUIControl.h>
#include <rendering/RenderControl.h>
#include "rendering/ViewFrustum.h"
#include "scene/VertexGroupManager.h"
#include "ui/qt/QGroupsListWidget.h"
#include "simulation/forces/LinearForce.h"
#include <iostream>
#include <map>
#include <math.h>
#include <memory>
#include <QVector3D>
#include <times/timing.h>

using namespace Eigen;

UIControl::UIControl()
{
    mViewFrustum = nullptr;
    mVgManager = new VertexGroupManager();
    mInteractionMode = InteractionMode::SELECT;
    mVgIdCounter = 0;
    mRenderControl = nullptr;
    mCurrentFolderPath = QDir::homePath().toStdString();
}

UIControl::~UIControl()
{
    delete mViewFrustum;
    delete mVgManager;
}

void UIControl::initialize(ApplicationControl* applicationControl)
{
    mAc = applicationControl;
    mViewFrustum = new ViewFrustum();

    // main window
    mMainWindow = new MainWindow(mAc);
    mMainWindow->setUIControl(this);
    mMainWindow->show();

    mGlWidget = mMainWindow->getGlWidget();

    initializeRenderer();

    mSelectionControl = std::make_unique<SelectionControl>(applicationControl, *mViewFrustum);
    mSelectionControl->init(mRenderControl->getRenderer());
    mSelectionControl->changeSelectionType(SelectionControl::SelectionType::SELECT_VERTICES);

    mSGQtWidgetManager = new SGQtWidgetManager(mMainWindow->getSGTreeWidget());
    mMainWindow->setSGQtWidgetManager(mSGQtWidgetManager);
    mModulesUIControl = std::make_unique<ModulesUIControl>(mMainWindow->getModulesParentWidget());
    mMainWindow->getModulesParentWidget()->layout()->addWidget(mModulesUIControl->getModulesWidget());
    handleNewSceneGraph();
}

void UIControl::initializeRenderer()
{
    glewInit();
    mGlWidget = mMainWindow->getGlWidget();
    mRenderControl = std::make_shared<RenderControl>(mGlWidget);
    mRenderControl->getRenderer()->initialize();
    mGlWidget->setRenderer(mRenderControl->getRenderer());
}

void UIControl::handleNewSceneGraph()
{
    // delete old ui

    // initiate SGQtWidgetManager
    mSGQtWidgetManager->registerNewSceneGraph(mAc->getSGControl()->getSceneGraph());
}

void UIControl::connectSignals()
{
    connectSignals(*(mMainWindow->getGlWidget()));
}

void UIControl::keyPressEvent(QKeyEvent* keyEvent)
{
    KeyManager::instance()->keyPressEvent(keyEvent);
    if (keyEvent->key() == Qt::Key::Key_Space)
    {
        // stop or start simulation
        mAc->getSimulationControl()->setSimulationPaused(
                    !mAc->getSimulationControl()->isSimulationPaused());
    }

    mGlWidget->updateCameraSpeedAfterKeyPress();
}

void UIControl::keyReleaseEvent(QKeyEvent* keyEvent)
{
    KeyManager::instance()->keyReleaseEvent(keyEvent);

    mGlWidget->updateCameraSpeedAfterKeyPress();
}

void UIControl::connectSignals(GLWidget& glWidget)
{

    QObject::connect(this, SIGNAL(repaintSignal()), &glWidget, SLOT(update()));
}


void UIControl::registerNewSimulation(Simulation* /*sim*/)
{

}

void UIControl::repaint()
{
    emit repaintSignal();
//    std::cout << "update()\n";
    //    mGlWidget->update();
}

void UIControl::revalidateTreeWidget()
{
    mSGQtWidgetManager->revalidate();
}

void UIControl::mousePressEvent(QMouseEvent *event)
{
    mMousePosPrev = event->pos();
    switch (mInteractionMode)
    {
    case SELECT:
    {
        if (event->button() == Qt::RightButton)
        {
            int x = event->pos().x();
            int y = mMainWindow->getGlWidget()->height() - event->pos().y();
            mSelectionControl->initiateNewSelection(x, y);

            // Cast ray and print information about the hit triangle.
            SGLeafNode* leafNode;
            std::shared_ptr<AbstractPolygon> poly;
            size_t triangleId;
            Eigen::Vector3d intersectionPoint;

            Eigen::Vector3f pos = mGlWidget->getCameraPos();

            Eigen::Vector3d screenPoint;
            gluUnProject(x, y, 0,
                         mViewFrustum->getModelView(),
                         mViewFrustum->getProjection(),
                         mViewFrustum->getViewPort(),
                         &screenPoint(0), &screenPoint(1), &screenPoint(2));

            bool hit = mAc->getSGControl()->castRay(
                        pos.cast<double>(), (screenPoint - pos.cast<double>()).normalized(), &leafNode, poly, triangleId, intersectionPoint);

            if (hit)
            {
                std::cout << "Ray: \n"
                          << "  node: " << leafNode->getName() << "\n"
                          << "  triangle id: " << triangleId << "\n"
                          << "  intersectionPoint: " << intersectionPoint.transpose() << "\n"
                          << "  distance: " << (pos.cast<double>() - intersectionPoint).norm() << "\n";
            }
            else
            {
                std::cout << "No Ray Hit\n";
            }
        }
        break;
    }
    case MOVE:
    {
        if (event->button() == Qt::RightButton)
        {
//            double fovy = 65.0;
//            double width = mViewFrustum->getViewPort()[2];
//            double height = mViewFrustum->getViewPort()[3];
//            double aspect = width / height;
//            double near = 0.1;
//            double far = 10000;

//            double proj[16];
            // TODO: use correct perspective

        }
        break;
    }
    case ACT_FORCE:
    {

        break;
    }
    case END:
    {

        break;
    }
    }
    // call gui with selection started event
}

void UIControl::mouseReleaseEvent(QMouseEvent *event)
{
    if (!mAc->getSGControl())
        return;

    switch (mInteractionMode)
    {
    case SELECT:
    {
        if (event->button() == Qt::RightButton)
        {
//            int x = event->pos().x();
//            int y = mMainWindow->getGlWidget()->height() - event->pos().y();

            mSelectionControl->finalizeSelection(*mViewFrustum);

//            mSelectionControl->updateSelectionRectanlge(x, y);
//            std::cout << "("
//                      << mSelectionControl->getSelectionRectangle()->getXStart()<< ", "
//                      << mSelectionControl->getSelectionRectangle()->getYStart() << ") ("
//                      << mSelectionControl->getSelectionRectangle()->getXEnd() << ", "
//                      << mSelectionControl->getSelectionRectangle()->getYEnd() << ")\n";
//            mSelectionControl->getSelection()->clear();
            // TODO: update selection for all scene leaf data


//            SGTraverser traverser = SGTraverserFactory::createDefaultSGTraverser(
//                        mAc->getSGControl()->getSceneGraph()->getRoot());
//            SGNodeVisitor* printerVisitor = SGNodeVisitorFactory::createPrinterVisitor(std::cout);
//            traverser.traverse(*printerVisitor);
//            delete printerVisitor;

        }
        break;
    }
    case MOVE:
    {
        // do nothing
        break;
    }
    case ACT_FORCE:
    {
        // calculate average vertex
        Vector avg( 0.0, 0.0, 0.0 );
        size_t count = 0;

        for (auto& it :
             mSelectionControl->getSelectionVertices()->getDataVectorsMap())
        {
            // linear force acts from each selected vertex to the point where clicked
            // where the z coordinate of the clicked point is the average of z coordinates
            // of selected point

            // alternaive: z coordinate of each point. This is kinda ugly though because it
            // would create many different destination points when many veritvces are selected.

            bool allSuccessful = true;
            for (ID vID : it.second)
            {
                std::shared_ptr<SimulationObject> so = it.first->getSimulationObject();
                if (so)
                {
                    avg += it.first->getSimulationObjectRaw()->getPosition(vID);
                    count++;
                }
                else
                {
                    allSuccessful = false;
                }
            }

            if (!allSuccessful)
                std::cout << "Attempted to add linear force to node that was not part of simulation.\n";
        }
        avg /= count;

        // create a geometric point/ simulation point, and reference it with
        // SimulationPointRef
        for (auto& it :
             mSelectionControl->getSelectionVertices()->getDataVectorsMap())
        {
            // linear force acts from each selected vertex to the point where clicked
            // where the z coordinate of the clicked point is the average of z coordinates
            // of selected point

            // alternaive: z coordinate of each point. This is kinda ugly though because it
            // would create many different destination points when many veritvces are selected.

            for (ID vID : it.second)
            {
                std::shared_ptr<SimulationObject> so = it.first->getSimulationObject();
                if (!so)
                    continue;

                // create a simulation point for now
                // TODO: remove the simulation point when the linear force is removed

                // TODO: I think, this is still way too ugly
                SGControl* sgControl = mAc->getSGControl();
                SGLeafNode* leafNode =
                        sgControl->createAndAddLeafNodeToRoot("linearForcePoint");
                leafNode->setData(std::make_shared<SceneLeafData>(leafNode));

                // calculate point where mouse is clicked
                // unproject mouse position with origin point z position
                int x = event->pos().x();
                int y = mMainWindow->getGlWidget()->height() - event->pos().y();

                // project avg point to obtain projected z coordinate
                double xWin, yWin, zWin;
                gluProject(avg(0), avg(1), avg(2),
                           mViewFrustum->getModelView(),
                           mViewFrustum->getProjection(),
                           mViewFrustum->getViewPort(),
                           &xWin, &yWin, &zWin);

                double xProj, yProj, zProj;
                gluUnProject(x,y,zWin,
                             mViewFrustum->getModelView(),
                             mViewFrustum->getProjection(),
                             mViewFrustum->getViewPort(),
                             &xProj, &yProj, &zProj);


                leafNode->getData()->setGeometricData(
                            std::make_shared<GeometricPoint>(
                                Vector(xProj, yProj, zProj)));
                sgControl->createAndSetCorrespondingSimulationObject(
                            leafNode);

                mAc->getSimulationControl()->addLinearForce(
                    it.first->getSimulationObjectRaw(), vID,
                    leafNode->getData()->getSimulationObjectRaw(), 0,
                    10.0);

            }
        }
        break;
    }
    case END:
    {
        break;
    }
    }
}

void UIControl::mouseMoveEvent(QMouseEvent *event)
{
    switch (mInteractionMode)
    {
    case SELECT:
    {
        if (event->buttons().testFlag(Qt::RightButton))
        {
            int x = event->pos().x();
            int y = mMainWindow->getGlWidget()->height() - event->pos().y();
            mSelectionControl->updateSelection(x, y);
            mMainWindow->getGlWidget()->update();
        }
        break;
    }
    case MOVE:
    {
        if (event->buttons() == Qt::RightButton)
        {
            double x = event->pos().x();
            double y = mMainWindow->getGlWidget()->height() - event->pos().y();
//            double z = 1; // near plane

            // move every vertex by the same amount
            Vector avg = Vector::Zero();
            size_t count = 0;
            for (auto it :
                 mSelectionControl->getSelectionVertices()->getDataVectorsMap())
            {
                std::shared_ptr<GeometricData> gd =
                        it.first->getGeometricData();

                for (ID id : it.second)
                {
                    avg += gd->getPosition(id);
                    count++;
                }
            }

            // use average vertex as reference point
            avg /= count;

            double xWin;
            double yWin;
            double zWin; // [0, 1], 0 = near plane, 1 = far plane

            gluProject(avg(0), avg(1), avg(2),
                       mViewFrustum->getModelView(),
                       mViewFrustum->getProjection(),
                       mViewFrustum->getViewPort(),
                       &xWin, &yWin, &zWin);

            x = xWin + x - mMousePosPrev.x();
            y = yWin + y - ( mMainWindow->getGlWidget()->height() - mMousePosPrev.y());

            double xProj;
            double yProj;
            double zProj;

            gluUnProject(x,y,zWin,
                         mViewFrustum->getModelView(),
                         mViewFrustum->getProjection(),
                         mViewFrustum->getViewPort(),
                         &xProj, &yProj, &zProj);

//            std::cout << "(" << avg(0) << ", " << avg(1) << ", " << avg(2) << ") => ("
//                      << xProj << ", " << yProj << ", " << zProj << ")\n";

            Vector avgTo(xProj, yProj, zProj);
            Vector avgDir = avgTo - avg;

            // Position update of vertex.
            // If node is part of simulation, movement of it will be done in
            // the simulation thread to guarantee consistency.
            // If its not part of simulation, movement is done immediately.
            for (auto it :
                 mSelectionControl->getSelectionVertices()->getDataVectorsMap())
            {
                if (it.first->getSimulationObjectRaw())
                {
                    switch (it.first->getSimulationObjectRaw()->getType())
                    {
                    case SimulationObject::Type::SIMULATION_POINT:
                    case SimulationObject::Type::RIGID_BODY:
                    {
                        SimulationObjectProxy(it.first->getSimulationObjectRaw()).addToPosition(avgDir, 0);
                        break;
                    }
                    case SimulationObject::Type::FEM_OBJECT:
                    {
                        for (ID id : it.second)
                        {
                            SimulationObjectProxy(it.first->getSimulationObjectRaw()).addToPosition(avgDir, id);
                        }
                        break;
                    }
                    }
                }
                else
                {
                    std::shared_ptr<GeometricData> gd = it.first->getGeometricData();
                    if (gd->getType() == GeometricData::Type::POLYGON)
                    {
                        std::shared_ptr<AbstractPolygon> poly = std::static_pointer_cast<AbstractPolygon>(gd);
                        poly->translate(avgDir);
                        poly->update();
                    }
                }
            }

        }
        break;
    }
    case ACT_FORCE:
    {
        break;
    }
    case END:
    {
        break;
    }
    }

    mMousePosPrev = event->pos();
}

void UIControl::handleProjectionUpdated()
{
//    if (mAc->getSimulationControl()->getFEMSimulation())
//    {
        glGetDoublev( GL_MODELVIEW_MATRIX, mViewFrustum->getModelView() );
        glGetDoublev( GL_PROJECTION_MATRIX, mViewFrustum->getProjection() );
        glGetIntegerv( GL_VIEWPORT, mViewFrustum->getViewPort() );

//        int* VP = mViewFrustum->getViewPort();

//        // project 3d point on 2d point
//    }
}

void UIControl::handlePreRenderingStep()
{
    START_TIMING_RENDERING("UIControl::handlePreRenderingStep()")
    if (mRenderControl)
    {
        START_TIMING_RENDERING("Renderer::handlePreRenderingStep()")
        mRenderControl->handlePreRenderingStep();
        STOP_TIMING_RENDERING
        START_TIMING_RENDERING("Other handlePreRenderingStep()")
        mAc->handlePreRenderingStep();
        if (mSelectionControl)
            mSelectionControl->updateModels();
        STOP_TIMING_RENDERING
//    if (mSelectionControl && mSelectionControl->getSelectionModel())
//        mSelectionControl->getSelectionModel()->updatePoints();
    }
    STOP_TIMING_RENDERING
}

void UIControl::handleCreateGroupButtonPressed()
{

}

void UIControl::handleActForceButtonPressed()
{
    // TODO: implement this
}

void UIControl::handleRemoveGroupButtonPressed()
{
    // TODO: implement this
}

void UIControl::loadFiles(const std::vector<File>& files, SGChildrenNode* parent)
{
    // Group all files that have identical names but the three different
    // extensions: node, tet, and ele. Pairs of (node, face) represent
    // a Polygon2D and pairs of (node, ele) a Polygon3D.

    struct NodeTetEle
    {
        const File* node = nullptr;
        const File* face = nullptr;
        const File* ele = nullptr;
    };

    std::map<std::string, NodeTetEle> nodeMap;
    for (const File& f : files)
    {
        std::string name = f.getName();
        if (f.getExtension() == ".node")
        {
            auto it = nodeMap.find(name);
            if (it != nodeMap.end())
            {
                it->second.node = &f;
            }
            else
            {
                NodeTetEle el;
                el.node = &f;
                nodeMap[name] = el;
            }
        }
        else if (f.getExtension() == ".face")
        {
            auto it = nodeMap.find(name);
            if (it != nodeMap.end())
            {
                it->second.face = &f;
            }
            else
            {
                NodeTetEle el;
                el.face = &f;
                nodeMap[name] = el;
            }
        }
        else if (f.getExtension() == ".ele")
        {
            auto it = nodeMap.find(name);
            if (it != nodeMap.end())
            {
                it->second.ele = &f;
            }
            else
            {
                NodeTetEle el;
                el.ele = &f;
                nodeMap[name] = el;
            }
        }
    }

    for (auto it : nodeMap)
    {
        if (it.second.node && (it.second.face || it.second.ele))
        {
            std::vector<File> files;
            files.push_back(*it.second.node);
            if (it.second.face)
                files.push_back(*it.second.face);
            if (it.second.ele)
                files.push_back(*it.second.ele);
            mAc->getSGControl()->importFilesAsChild(files, parent);
        }
    }

    // Create new leaf nodes that shares the name with the imported file
    for (const File& f : files)
    {
        if (f.getExtension() != ".node" &&
                f.getExtension() != ".face" &&
                f.getExtension() != ".ele")
        {
            mAc->getSGControl()->importFileAsChild(f, parent);
        }
    }
}

ViewFrustum* UIControl::getViewFrustum()
{
    return mViewFrustum;
}

RenderControl* UIControl::getRenderControl()
{
    return mRenderControl.get();
}

Renderer* UIControl::getRenderer()
{
    if (mRenderControl)
        return mRenderControl->getRenderer();
    return nullptr;
}

ModulesUIControl* UIControl::getModulesUIControl()
{
    return mModulesUIControl.get();
}

SelectionControl* UIControl::getSelectionControl()
{
    return mSelectionControl.get();
}

QWidget* UIControl::getModulesWidget()
{
    return mModulesUIControl->getModulesWidget();
}

void UIControl::onGroupItemClicked(QListWidgetItem *item)
{
    if (QGuiApplication::keyboardModifiers().testFlag(Qt::KeyboardModifier::ControlModifier))
    {

    }
    else
    {
        mSelectionControl->clearSelection();
    }

    VertexGroup* vg = mVgManager->getVertexGroup(static_cast<ID>(item->data(Qt::UserRole).toInt()));
    for (auto& it : vg->getDataVectorsMap())
    {
        for (ID vID : it.second)
        {
            mSelectionControl->getSelectionVertices()->addVertex(it.first, vID);
        }
    }
}

void UIControl::onGroupItemEntered(QListWidgetItem* /*item*/)
{
    // TODO: implement this
}

void UIControl::onCreateGroupActionTriggered()
{
    // create vertex group
    VertexGroup* vg = mVgManager->createVertexGroup(
                mSelectionControl->getSelectionVertices()->getDataVectorsMap());

//    mSelectionControl->getSelection()->getSelectedVertexGroups().insert(vg);

    // add corresponding iteam to ui
    mMainWindow->getGroupsListWidget()->addVertexGroup(static_cast<int>(vg->getId()));
//    mSelection->getSelectedVertices().clear();
}

void UIControl::onRemoveGroupActionTriggered()
{
    std::vector<int> selected_vgs = mMainWindow->getGroupsListWidget()
            ->getSelectedVertexGroupIds();
    for (int id : selected_vgs)
    {
        mVgManager->removeVertexGroup(static_cast<ID>(id));
    }
    mMainWindow->getGroupsListWidget()->removeSelectedVertexGroups();
}

void UIControl::onSelectActionTriggered(bool checked)
{
    if (checked)
        mInteractionMode = InteractionMode::SELECT;
}

void UIControl::onMoveGroupActionTriggered(bool checked)
{
    if (checked)
        mInteractionMode = InteractionMode::MOVE;
}

void UIControl::onActForceActionTriggered(bool checked)
{
    if (checked)
        mInteractionMode = InteractionMode::ACT_FORCE;
}

void UIControl::onAddTruncationActionTriggered()
{
    // TODO: code duplicatoin in SimulationUIControl
    // For now: add truncation
    mAc->getSimulationControl()->clearTruncations();
    const std::map<std::shared_ptr<SceneLeafData>, std::vector<ID>>& dvm =
            mAc->getUIControl()->getSelectionControl()->getSelectionVertices()
            ->getDataVectorsMap();
    for (std::map<std::shared_ptr<SceneLeafData>, std::vector<ID>>::const_iterator it = dvm.begin();
         it != dvm.end(); ++it)
    {
        if (!it->first->getSimulationObjectRaw() ||
            it->first->getSimulationObjectRaw()->getType() != SimulationObject::Type::FEM_OBJECT)
            continue;

        mAc->getSimulationControl()->addTruncations(
                    dynamic_cast<FEMObject*>(
                        it->first->getSimulationObjectRaw()), it->second);
    }
}

void UIControl::onAddNewSGNodeActionTriggered(QTreeWidgetItemWrapper* parentItem)
{

    // create new node and add it to parent
    class AddNewChildrenNodeVisitor : public SGNodeVisitor
    {
    public:
        AddNewChildrenNodeVisitor(UIControl& uic, QTreeWidgetItem* pi)
            : uiControl(uic)
            , parentItem(pi)
        {
        }

        virtual void visit(SGChildrenNode* childrenNode)
        {
            // SGNode
            childNode = new SGChildrenNode();
            childNode->setName("child");
            childrenNode->addChild(childNode);

            // Enable the editing to change the name of the node immediately
            // after adding it.
            uiControl.mSGQtWidgetManager->enableEditing(childNode);

            // QTreeWidgetItem
//            childItem = new QTreeWidgetItem(parentItem->treeWidget());
//            parentItem->addChild(childItem);

            // update SGQtWidgetManager
            //uiControl.mSGQtWidgetManager->addNewNode(childItem, childNode);
        }

        virtual void visit(SGLeafNode* /*leafNode*/)
        {

        }

        UIControl& uiControl;
        QTreeWidgetItem* parentItem;
        QTreeWidgetItem* childItem;
        SGChildrenNode* childNode;
    } visitor(*this, parentItem->getItem());

    // item is parent item
    SGNode* parentNode = mSGQtWidgetManager->get(parentItem->getItem());
    parentNode->accept(visitor);

}

void UIControl::onRemoveSGNodeActionTriggered(QTreeWidgetItemWrapper* item)
{
    SGNode* node = mSGQtWidgetManager->get(item->getItem());
    mAc->getSGControl()->removeNode(node);
}

void UIControl::onLoadFileSGNodeActionTriggered(QTreeWidgetItemWrapper* item)
{
    SGNode* node = mSGQtWidgetManager->get(item->getItem());
    class FileAsChildAdder : public SGNodeVisitor
    {
    public:
        FileAsChildAdder(UIControl& u)
            : ui(u)
        {
        }

        virtual void visit(SGChildrenNode* childrenNode)
        {
            // Start path dialog
            FileDialog fileDialog(ui.mMainWindow, ui.mCurrentFolderPath);
            std::vector<File> filePaths = fileDialog.getOpenFileNames(
                        "Load File", "*.ele *.face *.node *.obj *.off *.tet");
            if (!filePaths.empty())
                ui.mCurrentFolderPath = filePaths[0].getRelativePath();

            ui.loadFiles(filePaths, childrenNode);
        }

        virtual void visit(SGLeafNode* /*leafNode*/)
        {
            // Leaf nodes can't have children
        }

        UIControl& ui;
    } fileAdder(*this);

    node->accept(fileAdder);
}

void UIControl::onExportFileSGNodeActionTriggered(QTreeWidgetItemWrapper* item)
{
    SGNode* node = mSGQtWidgetManager->get(item->getItem());

    FileDialog fileDialog(mMainWindow, mCurrentFolderPath);
    File file = fileDialog.getSaveFileName(
                "Export File", "*.obj");

    if (file.getPath() != "")
    {
        mCurrentFolderPath = file.getRelativePath();

        mAc->getSGControl()->exportToSingleFile(file, node);
    }
}

void UIControl::onExportFilesSGNodeActionTriggered(QTreeWidgetItemWrapper* item)
{
    SGNode* node = mSGQtWidgetManager->get(item->getItem());
    FileDialog fileDialog(mMainWindow, mCurrentFolderPath);
    File file = fileDialog.getDirectory("Chose Target Directory");

    if (file.getPath() != "")
    {
        mCurrentFolderPath = file.getRelativePath();
        // TODO: Add option to specify target type

        mAc->getSGControl()->exportToMultipleFiles(file, ".obj", node);
    }
}

void UIControl::onSGSelectionChanged(QList<QTreeWidgetItem*>& selectedItems)
{
    // update SGQtWidgetManager
    for (QTreeWidgetItem* item : selectedItems)
    {
        SGNode* node = mSGQtWidgetManager->get(item);
        mSelectionControl->selectSceneNode(node);
    }
}

void UIControl::onSGSelectionChanged(std::set<SceneData*>& /*selectedItems*/)
{
    // called after selecting scene data in opengl window
    // update SelectionControl
}

void UIControl::onSelectionTypeChanged(int typeIndex)
{
    switch (typeIndex)
    {
    case 0:
        mSelectionControl->changeSelectionType(
                    SelectionControl::SELECT_VERTICES);
        break;
    case 1:
        mSelectionControl->changeSelectionType(
                    SelectionControl::SELECT_SCENE_NODES);
        break;
    }
}

void UIControl::onItemChanged(QTreeWidgetItem* item)
{
    // Name changed after editing the item in the scene graph.
    SGNode* node = mSGQtWidgetManager->get(item);
    if (node)
        node->setName(item->text(0).toStdString());
}

void UIControl::onSimulateActionTriggered(bool checked)
{
    mAc->getSimulationControl()->setSimulationPaused(!checked);
}

void UIControl::onMeshConverterActionTriggered()
{
    // start the mesh converter window,
    // create own mesh converter controller that
    // handles ui interaction and parts of the logic
}
