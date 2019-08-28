#ifndef UICONTROLL_H
#define UICONTROLL_H

#include <QObject>
#include <scene/scene_graph/SGCore.h>
#include <QPointF>
#include <set>
#include <ui/scene_graph/QTreeWidgetItemWrapper.h>

class ApplicationControl;
class GLWidget;
class LinearForce;
class MainWindow;
class ModulesUIControl;
class QMouseEvent;
class QKeyEvent;
class QListWidgetItem;
class QTreeWidgetItem;
class RenderControl;
class Renderer;
class RenderScreenRectangle;
class SelectionControl;
class SGControl;
class SGUIControl;
class Simulation;
class SimulationControl;
class SGQtWidgetManager;
class VertexGroup;
class ViewFrustum;
class VertexGroupManager;

class UIControl : public QObject
{
    Q_OBJECT

public:
    enum InteractionMode
    {
        SELECT, MOVE, ACT_FORCE, END
    };

    UIControl();

    ~UIControl();

    void initialize(ApplicationControl* applicationControl);
    void initializeRenderer();

    // Updates ui according to the provided scene graph.
    // Overrides changes from the previous scene graph.
    void handleNewSceneGraph();

    // call this method when simulation control is initialized and ready
    // to be called by the UI control
    void connectSignals();

    void keyPressEvent(QKeyEvent* keyEvent);

    void setSimulationControl(SimulationControl* simulationControl);

    void connectSignals(GLWidget& glWidget);


    void registerNewSimulation(Simulation* sim);

    void repaint();

    virtual void mousePressEvent(QMouseEvent* event);
    virtual void mouseReleaseEvent(QMouseEvent* event);
    virtual void mouseMoveEvent(QMouseEvent* event);

    void handleProjectionUpdated();

    // Rendering
    // Is called before each rendering step.
    void handlePreRenderingStep();

    // Buttons
    void handleCreateGroupButtonPressed();
    void handleActForceButtonPressed();
    void handleRemoveGroupButtonPressed();

    // Getter
    ViewFrustum* getViewFrustum();
    RenderControl* getRenderControl();
    Renderer* getRenderer();
    ModulesUIControl* getModulesUIControl();
    SelectionControl* getSelectionControl();
    // Returns widget that holds modules tabs, aka their parents.
    QWidget* getModulesWidget();

public slots:

    void onGroupItemClicked(QListWidgetItem* item);
    void onGroupItemEntered(QListWidgetItem* item);

    void onCreateGroupActionTriggered();
    void onRemoveGroupActionTriggered();
    void onSelectActionTriggered(bool checked);
    void onMoveGroupActionTriggered(bool checked);
    void onActForceActionTriggered(bool checked);
    void onAddTruncationActionTriggered();

    // Scene Graph Slots
        void onAddNewSGNodeActionTriggered(QTreeWidgetItemWrapper* item);
        void onRemoveSGNodeActionTriggered(QTreeWidgetItemWrapper* item);
        void onLoadFileSGNodeActionTriggered(QTreeWidgetItemWrapper* item);

        // Called by SelectionControl
        // Updates SGQtWidgetManager
        void onSGSelectionChanged(QList<QTreeWidgetItem*>& selectedItems);

        // Called by SGQtWidgetManager
        // Updates SelectionControl
        // TODO: implement this
        void onSGSelectionChanged(std::set<SceneData*>& selectedItems);

        void onSelectionTypeChanged(int typeIndex);

        void onItemChanged(QTreeWidgetItem* item);

    void onSimulateActionTriggered(bool checked);
    void onMeshConverterActionTriggered();
signals:
    void repaintSignal();

private:
    ApplicationControl* mAc;
    MainWindow* mMainWindow;

    GLWidget* mGlWidget;

    std::shared_ptr<RenderControl> mRenderControl;

    std::unique_ptr<SelectionControl> mSelectionControl;

    SGQtWidgetManager* mSGQtWidgetManager;
    std::unique_ptr<ModulesUIControl> mModulesUIControl;

    // the current view frustum
    // gets updated at the start of each rendering step.
    ViewFrustum* mViewFrustum;

    VertexGroupManager* mVgManager;
    std::vector<std::pair<VertexGroup*, LinearForce*>> mLinearForcePairs;
    //std::set<VertexGroup*> m_vertex_groups;

    InteractionMode mInteractionMode;
    QPointF mMousePosPrev;

    // vertex group id counter
    int mVgIdCounter;

};

#endif // UICONTROLL_H
