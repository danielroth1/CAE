#ifndef QTOWNERSMEMBERSWIDGET_H
#define QTOWNERSMEMBERSWIDGET_H

#include <map>
#include <memory>
#include <QObject>
#include <string>
#include <QWidget>

class QtMembersWidget;

// A QtOwnersMembersWidget stores multiple QtMembersWidget where each can
// be references by its unique name. The idea is that each of these
// widget has one owner and when updating that owner the widget can
// be identified by the owners name, e.g.
// QtOwnersMembersWidget* widget = new QtOwnersMembersWidget();
// widget->registerMembersWidget("FEMObject");
// widget->registerMembersWidget("PolygonRenderModel");
//
// std::vector<FEMObject*> selectedFEMObjects; <- fill this from the selection
// for (AbstractQtMemberWidget* w : widget->getMembersWidget("FEMObject")->getMemberWidgets())
// {
//      // these owners will be of type FEMObject* because this is the member widget
//      // with the name "FEMObject".
//      w->setOwners(selectedFEMObject);
// }
//
// -> only add MemberAccessors to the QtMembersWidget that have owner of the
//  type that is specified by this name.
//
// The order in which members widgets are registered is the order in which
// they are displayed in the UI.
//
class QtOwnersMembersWidget : public QWidget
{
    Q_OBJECT

public:
    QtOwnersMembersWidget(QWidget* parent = nullptr);

    // Updates the widget by removing all its elements and reinserting them.
    // Call this functions to make changes that were caused by the other
    // functions of this class take effect.
    void revalidate();

    // Registers a new QtMembersWidget and returns it. The widget can
    // also be accessed with getMembersWidget(const std::string&) by providing
    // the given name.
    // Chose a name that reflects the type of owners of MemberAccessors which
    // are added to the QtMembersWidget. Stick to that owner type! Don't add
    // different owners to a single QtMembersWidget because this would updating
    // all values of that QtMembersWidget by setting a single owner type impossible.
    //
    // Note: a QtMembersWidget can still have multiple owners as long as all
    // owners are of the same type.
    //
    // \param name - name of the QtMembersWidget which is used for access within
    //      this class.
    QtMembersWidget* registerMembersWidget(const std::string& name);

    // Sets the members widget with the given name visible.
    // \param name - the name that was used in registerMembersWidget()
    void setMembersWidgetVisible(const std::string& name, bool visible);

    // Returns the members widget with the given name.
    // \param name - the name that was used in registerMembersWidget()
    QtMembersWidget* getMembersWidget(const std::string& name);

    // Returns if the QtMembersWidget with the given name is visible. This
    // value can be inaccurate if the widgets visibility was changed wihtout
    // a revalidate() call afterwards.
    // \param name - the name that was used in registerMembersWidget()
    bool isVisible(const std::string& name);

private:
    struct MembersWidgetsProperties
    {
        QtMembersWidget* mMembersWidget;
        bool mVisible;
    };

    std::map<std::string, std::shared_ptr<MembersWidgetsProperties>> mPropertiesMap;
    std::vector<std::shared_ptr<MembersWidgetsProperties>> mProperties;

    // Properties that are added to the widget. After a revalidate() call this
    // is equivalent to mProperties.
    std::vector<std::shared_ptr<MembersWidgetsProperties>> mAddedProperties;
};

#endif // QTOWNERSMEMBERSWIDGET_H
