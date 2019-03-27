#ifndef DEMOLOADERUIFORM_H
#define DEMOLOADERUIFORM_H

#include <QWidget>

#include <memory>

#include <data_structures/BidirectionalMap.h>

class Demo;
class DemoLoaderUIControl;
class QTreeWidgetItem;

namespace Ui {
class DemoLoaderUIForm;
}

class DemoLoaderUIForm : public QWidget
{
    Q_OBJECT

public:
    explicit DemoLoaderUIForm(
            DemoLoaderUIControl* uiControl,
            QWidget *parent = nullptr);

    ~DemoLoaderUIForm();

    void addDemo(std::shared_ptr<Demo> demo);

    void removeDemo(std::shared_ptr<Demo> demo);

private slots:
    void on_mTreeWidgetDemos_itemDoubleClicked(QTreeWidgetItem *item, int column);

private:
    DemoLoaderUIControl* mUiControl;
    Ui::DemoLoaderUIForm* ui;

    BidirectionalMap<QTreeWidgetItem*, std::shared_ptr<Demo>> mBidirectionalMap;
};

#endif // DEMOLOADERUIFORM_H
