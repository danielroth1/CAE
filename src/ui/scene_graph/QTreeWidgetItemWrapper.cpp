#include "QTreeWidgetItemWrapper.h"


QTreeWidgetItemWrapper::QTreeWidgetItemWrapper(QTreeWidgetItem* item)
    : mItem(item)
{

}

QTreeWidgetItem* QTreeWidgetItemWrapper::getItem()
{
    return mItem;
}
