#ifndef QGROUPSLISTWIDGET_H
#define QGROUPSLISTWIDGET_H

#include <QListWidget>
#include <QObject>

class UIControl;

class QGroupsListWidget : public QListWidget
{
    Q_OBJECT
public:
    explicit QGroupsListWidget(QWidget *parent = 0);

    void connectSignals(UIControl* uiControl);

    void addVertexGroup(int id);

    std::vector<int> getSelectedVertexGroupIds();

    void removeSelectedVertexGroups();
    // not implemented yet
    void removeVertexGroup(int id);
};

#endif // QGROUPSLISTWIDGET_H
