#include "main_window.h"
#include "ui_mainwindow.h"
#include <QActionGroup>
#include <QSignalMapper>

#include "ui/UIControl.h"
#include "ApplicationControl.h"
#include <ui/scene_graph/QTreeWidgetItemWrapper.h>

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

GLWidget* MainWindow::getGlWidget()
{
    return ui->openGLWidget;
}

void MainWindow::insertNewModule(QWidget* moduleWidget, std::string name)
{
    ui->tabWidget->addTab(moduleWidget, QString::fromStdString(name));
}

void MainWindow::closeEvent(QCloseEvent* event)
{
    mAc->onExit();
    QMainWindow::closeEvent(event);
}

void MainWindow::setUIControl(UIControl *uiControl)
{
    m_ui_control = uiControl;
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

QTabWidget* MainWindow::getModulesTabWidget()
{
    return ui->tabWidget;
}

void MainWindow::on_sceneGraphTreeWidget_customContextMenuRequested(
        const QPoint &pos)
{
    QTreeWidget* tree = ui->sceneGraphTreeWidget;
    QTreeWidgetItem* item = tree->itemAt(pos);

    // add SGNode action
    QAction* addSGNodeAction = new QAction("New Node", this);
    QSignalMapper* mSignalMapper = new QSignalMapper(this);
    connect(addSGNodeAction, SIGNAL(triggered()),
            mSignalMapper, SLOT(map()));
    connect(mSignalMapper, SIGNAL(mapped(QObject*)),
            this, SLOT(addSGNodeSlot(QObject*)));
    mSignalMapper->setMapping(addSGNodeAction, new QTreeWidgetItemWrapper(item));

    // remove SGNode action
    QAction* removeSGNodeAction = new QAction("Remove Node", this);
    mSignalMapper = new QSignalMapper(this);
    connect(removeSGNodeAction, SIGNAL(triggered()),
            mSignalMapper, SLOT(map()));
    connect(mSignalMapper, SIGNAL(mapped(QObject*)),
            this, SLOT(removeSGNodeSlot(QObject*)));
    mSignalMapper->setMapping(removeSGNodeAction, new QTreeWidgetItemWrapper(item));

    // load file SGNode action
    QAction* loadFileSGNodeAction = new QAction("Load File", this);
    mSignalMapper = new QSignalMapper(this);
    connect(loadFileSGNodeAction, SIGNAL(triggered()),
            mSignalMapper, SLOT(map()));
    connect(mSignalMapper, SIGNAL(mapped(QObject*)),
            this, SLOT(loadFileSGNodeSlot(QObject*)));
    mSignalMapper->setMapping(loadFileSGNodeAction, new QTreeWidgetItemWrapper(item));

    // show custom context menu
    QMenu contextMenu;
    contextMenu.addAction(addSGNodeAction);
    contextMenu.addAction(removeSGNodeAction);
    contextMenu.addAction(loadFileSGNodeAction);

    contextMenu.exec( tree->mapToGlobal(pos) );
}

void MainWindow::addSGNodeSlot(QObject* parentNode)
{
    m_ui_control->onAddNewSGNodeActionTriggered(
                (QTreeWidgetItemWrapper*) parentNode);
}

void MainWindow::removeSGNodeSlot(QObject* node)
{
    m_ui_control->onRemoveSGNodeActionTriggered(
                (QTreeWidgetItemWrapper*) node);
}

void MainWindow::loadFileSGNodeSlot(QObject* node)
{
    m_ui_control->onLoadFileSGNodeActionTriggered(
                (QTreeWidgetItemWrapper*) node);
}

void MainWindow::on_actionSimulate_triggered(bool checked)
{
    m_ui_control->onSimulateActionTriggered(checked);
}

void MainWindow::on_actionMesh_Converter_triggered()
{
    m_ui_control->onMeshConverterActionTriggered();
}

void MainWindow::on_sceneGraphTreeWidget_itemChanged(QTreeWidgetItem* /*item*/, int /*column*/)
{

}

void MainWindow::on_sceneGraphTreeWidget_itemSelectionChanged()
{
    QTreeWidget* tree = ui->sceneGraphTreeWidget;
    QList<QTreeWidgetItem*> selectedItems = tree->selectedItems();
    m_ui_control->onSGSelectionChanged(selectedItems);
}

void MainWindow::on_mSelectionTypeComboBox_currentIndexChanged(int index)
{
    m_ui_control->onSelectionTypeChanged(index);
}

void MainWindow::on_actionClose_2_triggered()
{
    QApplication::quit();
}
