#ifndef QTMEMBERSWIDGET_H
#define QTMEMBERSWIDGET_H

#include <memory>
#include <QWidget>

#include <utils/MemberAccessor.h>

#include <Eigen/Core>


class AbstractQtMemberWidget;
class QGridLayout;

// A powerfull widget that features functionality to adapt the value
// of variables (global or class members) of various types.
//
// - use updateValue() (not update()) to update the values of each field
// in the ui.
class QtMembersWidget : public QWidget
{
    Q_OBJECT

public:
    QtMembersWidget(QWidget* parent = nullptr);

    virtual ~QtMembersWidget();

    // Updates the MemberWidgets by calling their update() method. This writes
    // their current values to their respective ui fields. This causes no
    // update of the actual member value (i.e. the ui update signals for
    // the ui elements, e.g. check box for bool, are surpressed).
    //
    // Note: update() calls the QWidgets update method and doesn't update
    // any field values.
    //
    void updateValues();

    void addBool(
            std::string name,
            const std::shared_ptr<MemberAccessorInterface<bool>>& memberAccessor);

    void addInteger(
            std::string name,
            const std::shared_ptr<MemberAccessorInterface<int>>& memberAccessor,
            int min = 0,
            int max = 100,
            int singleStep = 1);

    void addDouble(
            std::string name,
            const std::shared_ptr<MemberAccessorInterface<double>>& memberAccessor,
            double min = 0.0,
            double max = 100.0,
            double singleStep = 1.0,
            int precision = 3);

    void addVectorDouble(
            std::string name,
            const std::shared_ptr<MemberAccessorInterface<Eigen::Vector3d>>& memberAccessor,
            double min = 0.0,
            double max = 100.0,
            double singleStep = 1.0,
            int precision = 3);

    void addEnumComboBox(
            const std::string& name,
            const std::shared_ptr<MemberAccessorInterface<int>>& memberAccessor,
            const std::vector<std::string>& enumNames);

    // Returns the owner of the first QtMemberWidget. If there is no
    // QtMemberWidget, returns nullptr.
//    void* getOwner();

    // Sets the given owner as owner of all QtMemberWidgtes.
    // Does not call update() to update the widgets entries for
    // the new owner.
//    void setOwner(void* owner);

    bool hasWidgets(void* owner);

    const std::vector<AbstractQtMemberWidget*>& getMemberWidgets();

    void clearOwners();
    void addOwner(void* owner);

private:

    QGridLayout* mLayout;

    std::vector<AbstractQtMemberWidget*> mMemberWidgets;
};

#endif // QTMEMBERSWIDGET_H
