#include "main_window.h"
#include "ui_mainwindow.h"
#include <QActionGroup>
#include <QSignalMapper>

#include "ui/UIControl.h"
#include "ApplicationControl.h"
#include <ui/scene_graph/QTreeWidgetItemWrapper.h>

#include <QKeyEvent>

#include <ui/scene_graph/SGQtWidgetManager.h>


MainWindow::MainWindow(ApplicationControl* ac, QWidget* parent)
    : QMainWindow(parent)
    , mAc(ac)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->openGLWidget->setFocusPolicy(Qt::FocusPolicy::StrongFocus);

    QActionGroup* toolbarGroup = new QActionGroup(this);
    toolbarGroup->addAction(ui->actionCreate_Group);
    toolbarGroup->addAction(ui->actionEdit_Group);
    toolbarGroup->addAction(ui->actionRemove_Group);
    toolbarGroup->addAction(ui->actionSelect);
    toolbarGroup->addAction(ui->actionMove_Group);
    toolbarGroup->addAction(ui->actionActForce);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::setSGQtWidgetManager(SGQtWidgetManager* sgQtWidgetManager)
{
    mSGQtWidgetManager = sgQtWidgetManager;
}

GLWidget* MainWindow::getGlWidget()
{
    return ui->openGLWidget;
}

void MainWindow::closeEvent(QCloseEvent* event)
{
    mAc->onExit();
    QMainWindow::closeEvent(event);
}

void MainWindow::keyPressEvent(QKeyEvent* event)
{
    mUiControl->keyPressEvent(event);
}

void MainWindow::keyReleaseEvent(QKeyEvent* event)
{
    mUiControl->keyReleaseEvent(event);
}

void MainWindow::setUIControl(UIControl *uiControl)
{
    mUiControl = uiControl;
    ui->openGLWidget->setUIControl(uiControl);

    // connect singals
    QObject::connect(ui->actionCreate_Group, SIGNAL(triggered()),
                     uiControl, SLOT(onCreateGroupActionTriggered()));
    QObject::connect(ui->actionRemove_Group, SIGNAL(triggered()),
                     uiControl, SLOT(onRemoveGroupActionTriggered()));
    QObject::connect(ui->actionSelect, SIGNAL(triggered(bool)),
                     uiControl, SLOT(onSelectActionTriggered(bool)));
    QObject::connect(ui->actionMove_Group, SIGNAL(triggered(bool)),
                     uiControl, SLOT(onMoveGroupActionTriggered(bool)));
    QObject::connect(ui->actionActForce, SIGNAL(triggered(bool)),
                     uiControl, SLOT(onActForceActionTriggered(bool)));
    QObject::connect(ui->actionAddTruncation, SIGNAL(triggered()),
                     uiControl, SLOT(onAddTruncationActionTriggered()));
    ui->listWidget->connectSignals(uiControl);
}

QGroupsListWidget *MainWindow::getGroupsListWidget()
{
    return ui->listWidget;
}

QTreeWidget* MainWindow::getSGTreeWidget()
{
    return ui->sceneGraphTreeWidget;
}

QWidget* MainWindow::getModulesParentWidget()
{
    return ui->mModulesWidget;
}

void MainWindow::on_sceneGraphTreeWidget_customContextMenuRequested(
        const QPoint &pos)
{
    QTreeWidget* tree = ui->sceneGraphTreeWidget;
    QTreeWidgetItem* item = tree->itemAt(pos);
    SGNode* node = mSGQtWidgetManager->get(item);

    // renamd SGNode action
    QAction* renameSGNodeAction = new QAction("Rename", this);
    QSignalMapper* signalMapper = new QSignalMapper(this);
    connect(renameSGNodeAction, SIGNAL(triggered()),
            signalMapper, SLOT(map()));
    connect(signalMapper, SIGNAL(mapped(QObject*)),
            this, SLOT(renameSGNodeSlot(QObject*)));
    signalMapper->setMapping(renameSGNodeAction, new QTreeWidgetItemWrapper(item));

    // add SGNode action
    QAction* addSGNodeAction = new QAction("New Node", this);
    signalMapper = new QSignalMapper(this);
    connect(addSGNodeAction, SIGNAL(triggered()),
            signalMapper, SLOT(map()));
    connect(signalMapper, SIGNAL(mapped(QObject*)),
            this, SLOT(addSGNodeSlot(QObject*)));
    signalMapper->setMapping(addSGNodeAction, new QTreeWidgetItemWrapper(item));

    // remove SGNode action
    QAction* removeSGNodeAction = new QAction("Remove Node", this);
    signalMapper = new QSignalMapper(this);
    connect(removeSGNodeAction, SIGNAL(triggered()),
            signalMapper, SLOT(map()));
    connect(signalMapper, SIGNAL(mapped(QObject*)),
            this, SLOT(removeSGNodeSlot(QObject*)));
    signalMapper->setMapping(removeSGNodeAction, new QTreeWidgetItemWrapper(item));

    // load file SGNode action
    QAction* loadFileSGNodeAction = new QAction("Load File", this);
    signalMapper = new QSignalMapper(this);
    connect(loadFileSGNodeAction, SIGNAL(triggered()),
            signalMapper, SLOT(map()));
    connect(signalMapper, SIGNAL(mapped(QObject*)),
            this, SLOT(loadFileSGNodeSlot(QObject*)));
    signalMapper->setMapping(loadFileSGNodeAction, new QTreeWidgetItemWrapper(item));

    // Exporting single files is only supported for leaf nodes
    QAction* exportFileSGNodeAction = new QAction("Export File", this);
    signalMapper = new QSignalMapper(this);
    connect(exportFileSGNodeAction, SIGNAL(triggered()),
            signalMapper, SLOT(map()));
    connect(signalMapper, SIGNAL(mapped(QObject*)),
            this, SLOT(exportFileSGNodeSlot(QObject*)));
    signalMapper->setMapping(exportFileSGNodeAction, new QTreeWidgetItemWrapper(item));
    if (!node->isLeaf())
    {
        exportFileSGNodeAction->setEnabled(false);
    }

    QAction* exportFilesSGNodeAction = new QAction("Export Files", this);
    signalMapper = new QSignalMapper(this);
    connect(exportFilesSGNodeAction, SIGNAL(triggered()),
            signalMapper, SLOT(map()));
    connect(signalMapper, SIGNAL(mapped(QObject*)),
            this, SLOT(exportFilesSGNodeSlot(QObject*)));
    signalMapper->setMapping(exportFilesSGNodeAction, new QTreeWidgetItemWrapper(item));

    // show custom context menu
    QMenu contextMenu;
    contextMenu.addAction(renameSGNodeAction);
    contextMenu.addAction(addSGNodeAction);
    contextMenu.addAction(removeSGNodeAction);
    contextMenu.addAction(loadFileSGNodeAction);
    contextMenu.addAction(exportFileSGNodeAction);
    contextMenu.addAction(exportFilesSGNodeAction);

    contextMenu.exec( tree->mapToGlobal(pos) );
}

void MainWindow::renameSGNodeSlot(QObject* node)
{
    QTreeWidgetItemWrapper* item = static_cast<QTreeWidgetItemWrapper*>(node);
    ui->sceneGraphTreeWidget->editItem(item->getItem(), 0);
}

void MainWindow::addSGNodeSlot(QObject* parentNode)
{
    mUiControl->onAddNewSGNodeActionTriggered(
                static_cast<QTreeWidgetItemWrapper*>(parentNode));
}

void MainWindow::removeSGNodeSlot(QObject* node)
{
    mUiControl->onRemoveSGNodeActionTriggered(
                static_cast<QTreeWidgetItemWrapper*>(node));
}

void MainWindow::loadFileSGNodeSlot(QObject* node)
{
    mUiControl->onLoadFileSGNodeActionTriggered(
                static_cast<QTreeWidgetItemWrapper*>(node));
}

void MainWindow::exportFileSGNodeSlot(QObject* node)
{
    mUiControl->onExportFileSGNodeActionTriggered(
                static_cast<QTreeWidgetItemWrapper*>(node));
}

void MainWindow::exportFilesSGNodeSlot(QObject* node)
{
    mUiControl->onExportFilesSGNodeActionTriggered(
                static_cast<QTreeWidgetItemWrapper*>(node));
}

void MainWindow::on_actionSimulate_triggered(bool checked)
{
    mUiControl->onSimulateActionTriggered(checked);
}

void MainWindow::on_actionMesh_Converter_triggered()
{
    mUiControl->onMeshConverterActionTriggered();
}

void MainWindow::on_sceneGraphTreeWidget_itemChanged(
        QTreeWidgetItem* item, int /*column*/)
{
    mUiControl->onItemChanged(item);
}

void MainWindow::on_sceneGraphTreeWidget_itemSelectionChanged()
{
    QTreeWidget* tree = ui->sceneGraphTreeWidget;
    QList<QTreeWidgetItem*> selectedItems = tree->selectedItems();
    mUiControl->onSGSelectionChanged(selectedItems);
}

void MainWindow::on_mSelectionTypeComboBox_currentIndexChanged(int index)
{
    mUiControl->onSelectionTypeChanged(index);
}

void MainWindow::on_actionClose_2_triggered()
{
    QApplication::quit();
}
