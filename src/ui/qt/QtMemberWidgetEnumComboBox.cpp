#include "QtMemberWidgetEnumComboBox.h"

#include <QComboBox>
#include <QVBoxLayout>

QtMemberWidgetEnumComboBox::QtMemberWidgetEnumComboBox(
        const std::shared_ptr<MemberAccessorInterface<int>>& memberAccessor,
        QWidget* parent,
        const std::vector<std::string>& enumNames)
    : QtMemberWidget<int>(memberAccessor, parent)
{
    mComboBox = new QComboBox();
    for (const std::string& enumName : enumNames)
    {
        mComboBox->addItem(QString::fromStdString(enumName));
    }
//    mComboBox->addItem("");

    QVBoxLayout* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0); // no border

    layout->addWidget(mComboBox);

    // Connect the check box
    QObject::connect(mComboBox, SIGNAL(currentIndexChanged(int)),
                     this, SLOT(currentIndexChanged(int)));

}

QtMemberWidgetEnumComboBox::~QtMemberWidgetEnumComboBox()
{

}

void QtMemberWidgetEnumComboBox::currentIndexChanged(int index)
{
    mMemberAccessor->setData(index);
}

void QtMemberWidgetEnumComboBox::updateSlot()
{
    // Blocking the signals to prevent infinite callback loops.
    mComboBox->blockSignals(true);
    if (!isAccessorValuesIdentical())
    {
        mComboBox->setCurrentIndex(-1);
    }
    else
    {
        mComboBox->setCurrentIndex(mMemberAccessor->getData());
    }
    mComboBox->blockSignals(false);
}
