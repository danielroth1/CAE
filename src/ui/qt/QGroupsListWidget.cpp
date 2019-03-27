#include "QGroupsListWidget.h"

#include "ui/UIControl.h"
#include <iostream>

QGroupsListWidget::QGroupsListWidget(QWidget *parent)
    : QListWidget(parent)
{

}

void QGroupsListWidget::connectSignals(UIControl *uiControl)
{
    QWidget::connect(this, SIGNAL(itemClicked(QListWidgetItem*)),
                     uiControl, SLOT(onGroupItemClicked(QListWidgetItem*)));
    QWidget::connect(this, SIGNAL(itemEntered(QListWidgetItem*)),
                     uiControl, SLOT(onGroupItemEntered(QListWidgetItem*)));
}

void QGroupsListWidget::addVertexGroup(int id)
{
    QListWidgetItem* item = new QListWidgetItem();
    item->setData(Qt::UserRole, QVariant(id));
    addItem(item);
    item->setText("Group" + QString::number(id));
}

std::vector<int> QGroupsListWidget::getSelectedVertexGroupIds()
{
    std::vector<int> ids;
    ids.reserve(selectedItems().size());
    for (QListWidgetItem* item : selectedItems())
    {
        ids.push_back(item->data(Qt::UserRole).toInt());
    }
    return ids;

}

void QGroupsListWidget::removeSelectedVertexGroups()
{
    qDeleteAll(selectedItems());
}

void QGroupsListWidget::removeVertexGroup(int /*id*/)
{
    // TODO: implement this method
    // to do so, a map must be used that maps vertex group ids to QListWidgetItems
    // the map must be updated with every add and remove operation
}
