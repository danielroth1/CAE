#ifndef QTMEMBERSWIDGET_H
#define QTMEMBERSWIDGET_H

#include <memory>
#include <QWidget>

#include <utils/MemberAccessor.h>


class AbstractQtMemberWidget;
class QGridLayout;

// A powerfull widget that features functionality to adapt the value
// of variables (global or class members) of various types.
class QtMembersWidget : public QWidget
{
    Q_OBJECT

public:
    QtMembersWidget(QWidget* parent = nullptr);

    virtual ~QtMembersWidget();

    void addBool(
            std::string name,
            const std::shared_ptr<MemberAccessor<bool>>& memberAccessor);

    void addInteger(
            std::string name,
            const std::shared_ptr<MemberAccessor<int>>& memberAccessor);

    void addDouble(
            std::string name,
            const std::shared_ptr<MemberAccessor<double>>& memberAccessor);

private:
    QGridLayout* mLayout;

    std::vector<AbstractQtMemberWidget*> mMemberWidgets;
};

#endif // QTMEMBERSWIDGET_H
