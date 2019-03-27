#include "ModulesUIControl.h"

#include <QTabWidget>

#include <modules/Module.h>

ModulesUIControl::ModulesUIControl(QTabWidget* tabWidget)
    : mTabWidget(tabWidget)
{

}

bool ModulesUIControl::addModule(Module* module)
{
    module->initUI(mTabWidget);
    return addModule(module->getName(), module->getWidget());
}

bool ModulesUIControl::removeModule(Module* module)
{
    return removeModule(module->getName());
}

QWidget* ModulesUIControl::getModulesWidget()
{
    return mTabWidget;
}

bool ModulesUIControl::addModule(std::string name, QWidget* widget)
{
    if (findTabIndex(name) != -1)
        return false;

    mTabWidget->insertTab(mTabWidget->count(), widget, QString::fromStdString(name));
    mTabWidget->setCurrentIndex(0);
    return true;

//    // don't add module if there is already one with same name added
//    auto it = std::find(mModuleNames.begin(), mModuleNames.end(), name);
//    if (it == mModuleNames.end())
//    {
//        // create new tab
//        mTabWidget->addTab(widget, QString::fromStdString(name));

//        // add entry in vector that is at the same indx as the
//        // corresponding widget in mTabWidget
//        mModuleNames.push_back(name);
//    }
}

bool ModulesUIControl::removeModule(std::string name)
{
    int index = findTabIndex(name);
    if (index == -1)
        return false;

    mTabWidget->removeTab(index);
    return true;

//    // find entry in vector
//    auto it = std::find(mModuleNames.begin(), mModuleNames.end(), name);
//    // if there is an entry...
//    if (it != mModuleNames.end())
//    {
//        // obtain index of iterator
//        int index = static_cast<int>(it - mModuleNames.begin());
//        // remove tab ob module
//        mTabWidget->removeTab(index);
//        // remove entry in vector
//        mModuleNames.erase(it);
//    }
}

int ModulesUIControl::findTabIndex(std::string name)
{
    for (int i = 0; i < mTabWidget->count(); ++i)
    {

        if (mTabWidget->tabText(i).toStdString() == name)
        {
            return i;
        }
    }
    return -1;
}
