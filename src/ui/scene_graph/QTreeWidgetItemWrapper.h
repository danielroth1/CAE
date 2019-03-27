#ifndef QTREEWIDGETITEMWRAPPER_H
#define QTREEWIDGETITEMWRAPPER_H

#include <QTreeWidget>
#include <qobject.h>



class QTreeWidgetItemWrapper : public QObject
{
public:
    QTreeWidgetItemWrapper(QTreeWidgetItem* item);

    QTreeWidgetItem* getItem();

private:
    QTreeWidgetItem* mItem;
};

#endif // QTREEWIDGETITEMWRAPPER_H
