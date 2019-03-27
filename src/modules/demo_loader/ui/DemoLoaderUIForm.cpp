#include "DemoLoaderUIControl.h"
#include "DemoLoaderUIForm.h"
#include "ui_DemoLoaderUIForm.h"

#include <modules/demo_loader/Demo.h>

#include <QStringList>
#include <QTreeWidget>
#include <QTreeWidgetItem>


DemoLoaderUIForm::DemoLoaderUIForm(
        DemoLoaderUIControl* uiControl,
        QWidget *parent) :
    QWidget(parent),
    mUiControl(uiControl),
    ui(new Ui::DemoLoaderUIForm)
{
    ui->setupUi(this);
}

DemoLoaderUIForm::~DemoLoaderUIForm()
{
    delete ui;
}

void DemoLoaderUIForm::addDemo(std::shared_ptr<Demo> demo)
{
    QTreeWidgetItem* item = new QTreeWidgetItem(
                static_cast<QTreeWidget*>(nullptr),
                QStringList(QString::fromStdString(demo->getName())));

    ui->mTreeWidgetDemos->insertTopLevelItem(
                ui->mTreeWidgetDemos->topLevelItemCount(),
                item);

    mBidirectionalMap.add(item, demo);
}

void DemoLoaderUIForm::removeDemo(std::shared_ptr<Demo> demo)
{
    QTreeWidgetItem* item = mBidirectionalMap.get(demo);
    if (item)
    {
        ui->mTreeWidgetDemos->removeItemWidget(item, 0);
    }
}

void DemoLoaderUIForm::on_mTreeWidgetDemos_itemDoubleClicked(
        QTreeWidgetItem *item, int /*column*/)
{
    std::shared_ptr<Demo> demo = mBidirectionalMap.get(item);
    if (demo)
    {
        mUiControl->notifyDemoClicked(demo);
    }
}
