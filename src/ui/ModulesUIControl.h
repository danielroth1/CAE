#ifndef MODULESUICONTROL_H
#define MODULESUICONTROL_H

#include <string>
#include <vector>

class Module;
class QTabWidget;
class QToolBox;
class QWidget;

// Manages the UI interface to modules
// mModuleNames is always in sync with the underlying
// data structure of QT for tab widgets. Entries at an index i
// in mModuleNames are names of modules that are at the
// tab with index i.
// What happens if tabs are moved? Overthink this concept?
// -> don't use another data structure but iterate over the tabs themselfs instead?
// -> how about allowing a more dynamic ui?
class ModulesUIControl
{
public:
    ModulesUIControl(QWidget* modulesWidget);

    bool addModule(Module* module);
    bool removeModule(Module* module);

    // Returns widget that holds modules tabs, aka their parents.
    QWidget* getModulesWidget();


private:

    // Adds a module tab to the modules tab widget. Sets the
    // given widget as widget of the tab.
    // \return true if insertion was successfull. It is not succesfull if
    //      the module by the given name or another module with the same
    //      name was already added.
    bool addModule(std::string name, QWidget* widget);

    // Removes the module tab and its widget.
    // \return true if remove was successfull or false if there was no
    //      module with the given name that could have been removed.
    bool removeModule(std::string name);

    int findTabIndex(std::string name);

//    QTabWidget* mTabWidget;
    QToolBox* mToolBox;
    std::vector<std::string> mModuleNames;
//    std::map<std::string, int> mModuleWidgetMap;
};

#endif // MODULESUICONTROL_H
