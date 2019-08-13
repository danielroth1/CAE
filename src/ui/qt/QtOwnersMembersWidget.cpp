#include "QtMembersWidget.h"
#include "QtOwnersMembersWidget.h"

#include <iostream>
#include <QVBoxLayout>
#include <QLabel>
#include <QMetaObject>

QtOwnersMembersWidget::QtOwnersMembersWidget(QWidget* parent)
    : QWidget (parent)
{
    QVBoxLayout* layout = new QVBoxLayout(this);
    setLayout(layout);
}

void QtOwnersMembersWidget::revalidate()
{
    QMetaObject::invokeMethod(this, "revalidateSlot");
}

QtMembersWidget* QtOwnersMembersWidget::registerMembersWidget(
        const std::string& name)
{
    QtMembersWidget* membersWidget = new QtMembersWidget(
                static_cast<QWidget*>(parent()));

    membersWidget->layout()->setContentsMargins(0, 0, 0, 0);

    // add headline
    QLabel* nameLabel = new QLabel(membersWidget);
    nameLabel->setText(QString::fromStdString(name));
    QFont font(nameLabel->font());
    font.setBold(true);
    nameLabel->setFont(font);
    membersWidget->layout()->addWidget(nameLabel);

    std::shared_ptr<MembersWidgetsProperties> p =
            std::make_shared<MembersWidgetsProperties>();
    p->mVisible = true;
    p->mAdded = false;
    p->mMembersWidget = membersWidget;

    mPropertiesMap[name] = p;
    mProperties.push_back(p);

    return membersWidget;
}

void QtOwnersMembersWidget::setMembersWidgetVisible(const std::string& name,
                                                    bool visible)
{
    // alternative approach to setting each widget visible/ invisible.
    // Flag them as such and recreate the widget at once. This solves
    // the correct ordering and complicated removing of single widgets
    // which is probably not well supported.

    // remove the members widget from this widget
    auto it = mPropertiesMap.find(name);
    if (it != mPropertiesMap.end())
    {
        it->second->mVisible = visible;
    }
    else
    {
        std::cout << "No widget registered with the name " << name <<"\n";
    }

}

QtMembersWidget* QtOwnersMembersWidget::getMembersWidget(const std::string& name)
{
    return mPropertiesMap[name]->mMembersWidget;
}

bool QtOwnersMembersWidget::isVisible(const std::string& name)
{
    return mPropertiesMap[name]->mVisible;
}

void QtOwnersMembersWidget::revalidateSlot()
{
    // Add all widgets that are not already.
    for (const std::shared_ptr<MembersWidgetsProperties>& p : mProperties)
    {
        if (!p->mAdded)
        {
            layout()->addWidget(p->mMembersWidget);
            p->mAdded = true;
        }
    }

    // add them in the correct order
    for (const std::shared_ptr<MembersWidgetsProperties>& p : mProperties)
    {
        p->mMembersWidget->setVisible(p->mVisible);
    }
}
