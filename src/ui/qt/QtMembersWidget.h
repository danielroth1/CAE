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
            const std::shared_ptr<MemberAccessor<int>>& memberAccessor,
            int min = 0,
            int max = 100,
            int singleStep = 1);

    void addDouble(
            std::string name,
            const std::shared_ptr<MemberAccessor<double>>& memberAccessor,
            double min = 0.0,
            double max = 100.0,
            double singleStep = 1.0,
            int precision = 3);

    void addVectorDouble(
            std::string name,
            const std::shared_ptr<MemberAccessor<Eigen::Vector3d>>& memberAccessor,
            double min = 0.0,
            double max = 100.0,
            double singleStep = 1.0,
            int precision = 3);

private:
    QGridLayout* mLayout;

    std::vector<AbstractQtMemberWidget*> mMemberWidgets;
};

#endif // QTMEMBERSWIDGET_H
