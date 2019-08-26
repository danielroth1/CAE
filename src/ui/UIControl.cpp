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
#include <math.h>
#include <memory>
#include <times/timing.h>

using namespace Eigen;

UIControl::UIControl()
{
    mViewFrustum = nullptr;
    mVgManager = new VertexGroupManager();
    mInteractionMode = InteractionMode::SELECT;
    mVgIdCounter = 0;
    mRenderControl = nullptr;
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
    mModulesUIControl = std::make_unique<ModulesUIControl>(mMainWindow->getModulesTabWidget());
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

void UIControl::keyPressEvent(QKeyEvent *keyEvent)
{
    if (keyEvent->key() == Qt::Key::Key_Space)
    {
        // stop or start simulation
        mAc->getSimulationControl()->setSimulationPaused(
                    !mAc->getSimulationControl()->isSimulationPaused());
    }
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
            std::cout << "("
                      << mSelectionControl->getSelectionRectangle()->getXStart()<< ", "
                      << mSelectionControl->getSelectionRectangle()->getYStart() << ") ("
                      << mSelectionControl->getSelectionRectangle()->getXEnd() << ", "
                      << mSelectionControl->getSelectionRectangle()->getYEnd() << ")\n";
//            mSelectionControl->getSelection()->clear();
            // TODO: update selection for all scene leaf data


            SGTraverser traverser = SGTraverserFactory::createDefaultSGTraverser(
                        mAc->getSGControl()->getSceneGraph()->getRoot());
            SGNodeVisitor* printerVisitor = SGNodeVisitorFactory::createPrinterVisitor(std::cout);
            traverser.traverse(*printerVisitor);
            delete printerVisitor;

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

                double xProj, yProj, zProj; // TODO: some probleme here?
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

            mSelectionControl->updateSelection(
                        static_cast<int>(x),
                        static_cast<int>(y));

//            QVector3D camPos = mGlWidget->getCameraPos();
//            QVector3D camDir = mGlWidget->getCameraDir();

            // move every vertex by the same amount
            Vector avg = Vector::Zero();
            size_t count = 0;
            for (auto it :
                 mSelectionControl->getSelectionVertices()->getDataVectorsMap())
            {
                if (!it.first->getSimulationObjectRaw())
                    continue;

                for (ID id : it.second)
                {
                    // TODO: replace with simulation/geometry references
                    Vector& pos = it.first->getSimulationObjectRaw()->getPosition(id);
                    avg += pos;
                    count++;
                }
            }

            // use average vertex as reference point
            avg /= count;

            double xWin;
            double yWin;
            double zWin;

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

            // projects to far plane I think. What is actually desired is the plane
            // on which the vertex is
            // which vertex? there can be multiple selected
            // how to make that all selected vertices move in parallel
            //   -> determine moving direction
            // TODO: use a different function here
            gluUnProject(x,y,zWin,
                         mViewFrustum->getModelView(),
                         mViewFrustum->getProjection(),
                         mViewFrustum->getViewPort(),
                         &xProj, &yProj, &zProj);

    //                Vector n1(xProj, yProj, zProj);
    //                n1.normalize();

//            std::cout << "(" << avg(0) << ", " << avg(1) << ", " << avg(2) << ") => ("
//                      << xProj << ", " << yProj << ", " << zProj << ")\n";

            Vector avgTo(xProj, yProj, zProj);
            Vector avgDir = avgTo - avg;


//            for (auto it : mSelection->getDataVectorsMap())
//            {
//                for (ID id : it.second)
//                {
//                    // TODO: replace with simulation/ geometry reference
//                    Vector& pos = it.first->getSimulationObjectRaw()->getPosition(id);

//                    pos += avgDir;
//                }
//            }

            // Position update of vertex.
            // This should be modeled differently and should
            // at least be handled in the simulation class.
            for (auto it :
                 mSelectionControl->getSelectionVertices()->getDataVectorsMap())
            {
                if (!it.first->getSimulationObjectRaw())
                    continue;

                for (ID id : it.second)
                {
                    SimulationObjectProxy(it.first->getSimulationObjectRaw()).addToPosition(avgDir, id);
                }
            }

            // TODO: implement this
            // Selection
            // project selection correctly
            // call simulation to move vertices, all vertices by the same amount

        }
        break;
    }
    case ACT_FORCE:
    {
        // TODO: implement this
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
            FileDialog fileDialog(ui.mMainWindow);
            std::vector<File> filePaths = fileDialog.getOpenFileNames(
                        "Load File", "*.off *.obj *.tet");

            // Create new leaf nodes that shares the name with the imported file
            for (File& f : filePaths)
            {
                ui.mAc->getSGControl()->importFileAsChild(f, childrenNode);
            }
        }

        virtual void visit(SGLeafNode* /*leafNode*/)
        {
            // Leaf nodes can't have children
        }

        UIControl& ui;
    } fileAdder(*this);

    node->accept(fileAdder);
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
