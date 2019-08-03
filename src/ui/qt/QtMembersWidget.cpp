#include "QtMemberWidgetBool.h"
#include "QtMemberWidgetDouble.h"
#include "QtMemberWidgetInteger.h"
#include "QtMemberWidgetVectorDouble.h"
#include "QtMembersWidget.h"

#include <QGridLayout>
#include <QLabel>

QtMembersWidget::QtMembersWidget(QWidget* parent)
    : QWidget(parent)
{
    mLayout = new QGridLayout(this);
}

QtMembersWidget::~QtMembersWidget()
{

}

void QtMembersWidget::updateValues()
{
    for (AbstractQtMemberWidget* qtMemberWidget : mMemberWidgets)
    {
        qtMemberWidget->update();
    }
}

void QtMembersWidget::addBool(
        std::string name,
        const std::shared_ptr<MemberAccessor<bool>>& memberAccessor)
{
    int row = mLayout->rowCount();

    mLayout->addWidget(new QLabel(QString::fromStdString(name)), row, 0);

    QtMemberWidget<bool>* memberWidget =
            new QtMemberWidgetBool(memberAccessor, this);
    mMemberWidgets.push_back(memberWidget);

    mLayout->addWidget(memberWidget, row, 1);

    memberWidget->update();
}

void QtMembersWidget::addInteger(
        std::string name,
        const std::shared_ptr<MemberAccessor<int>>& memberAccessor,
        int min,
        int max,
        int singleStep)
{
    int row = mLayout->rowCount();
    mLayout->addWidget(new QLabel(QString::fromStdString(name)), row, 0);
    QtMemberWidget<int>* memberWidget =
            new QtMemberWidgetInteger(memberAccessor, this, min, max, singleStep);
    mMemberWidgets.push_back(memberWidget);
    mLayout->addWidget(memberWidget, row, 1);

    memberWidget->update();
}

void QtMembersWidget::addDouble(
        std::string name,
        const std::shared_ptr<MemberAccessor<double>>& memberAccessor,
        double min,
        double max,
        double singleStep,
        int precision)
{
    int row = mLayout->rowCount();
    mLayout->addWidget(new QLabel(QString::fromStdString(name)), row, 0);

    QtMemberWidget<double>* memberWidget =
            new QtMemberWidgetDouble(memberAccessor, this, min, max,
                                     singleStep, precision);
    mMemberWidgets.push_back(memberWidget);
    mLayout->addWidget(memberWidget, row, 1);

    memberWidget->update();
}

void QtMembersWidget::addVectorDouble(
        std::string name,
        const std::shared_ptr<MemberAccessor<Eigen::Vector3d>>& memberAccessor,
        double min,
        double max,
        double singleStep,
        int precision)
{
    int row = mLayout->rowCount();
    mLayout->addWidget(new QLabel(QString::fromStdString(name)), row, 0);

    QtMemberWidget<Eigen::Vector3d>* memberWidget =
            new QtMemberWidgetVectorDouble(memberAccessor, this, min, max,
                                           singleStep, precision);

    mMemberWidgets.push_back(memberWidget);
    mLayout->addWidget(memberWidget, row, 1);

    memberWidget->update();
}

const std::vector<AbstractQtMemberWidget*>& QtMembersWidget::getMemberWidgets()
{
    return mMemberWidgets;
}
