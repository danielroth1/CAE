#include "QtMemberWidgetInteger.h"

#include <QSpinBox>
#include <QVBoxLayout>

QtMemberWidgetInteger::QtMemberWidgetInteger(
        const std::shared_ptr<MemberAccessor<int>>& memberAccessor,
        QWidget* parent,
        int min,
        int max,
        int singleStep)
    : QtMemberWidget<int>(memberAccessor, parent)
{
    mSpinBox = new QSpinBox();

    QVBoxLayout* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0); // no border

    layout->addWidget(mSpinBox);

    // Connect the check box
    QObject::connect(mSpinBox, SIGNAL(valueChanged(int)),
                     this, SLOT(valueChanged(int)));

    mSpinBox->setRange(min, max);
    mSpinBox->setSingleStep(singleStep);
}

QtMemberWidgetInteger::~QtMemberWidgetInteger()
{

}

void QtMemberWidgetInteger::valueChanged(int value)
{
    mMemberAccessor->setData(value);
}

void QtMemberWidgetInteger::updateSlot()
{
    // Blocking the signals to prevent infinite callback loops.
    mSpinBox->blockSignals(true);
    mSpinBox->setValue(mMemberAccessor->getData());
    mSpinBox->blockSignals(false);
}
