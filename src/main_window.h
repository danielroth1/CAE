#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include <glwidget.h>

class ApplicationControl;
class UIControl;
class QGroupsListWidget;
class QTreeWidget;
class QTreeWidgetItem;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(ApplicationControl* ac, QWidget *parent = nullptr);
    ~MainWindow();

    GLWidget* getGlWidget();

    void setUIControl(UIControl* uiControl);

    QGroupsListWidget* getGroupsListWidget();
    QTreeWidget* getSGTreeWidget();
    QWidget* getModulesParentWidget();

    // QWidget interface
protected:
    virtual void closeEvent(QCloseEvent* event);
    virtual void keyPressEvent(QKeyEvent* event);
    virtual void keyReleaseEvent(QKeyEvent* event);

private slots:
    // none

    void on_sceneGraphTreeWidget_customContextMenuRequested(const QPoint &pos);

    void renameSGNodeSlot(QObject* node);
    void addSGNodeSlot(QObject* parentNode);
    void removeSGNodeSlot(QObject* node);
    void loadFileSGNodeSlot(QObject* node);

    void on_actionSimulate_triggered(bool checked);

    void on_actionMesh_Converter_triggered();

    void on_sceneGraphTreeWidget_itemChanged(QTreeWidgetItem *item, int column);

    void on_sceneGraphTreeWidget_itemSelectionChanged();

    void on_mSelectionTypeComboBox_currentIndexChanged(int index);

    void on_actionClose_2_triggered();

private:
    ApplicationControl* mAc;

    Ui::MainWindow *ui;

    UIControl* mUiControl;


};

#endif // MAINWINDOW_H
