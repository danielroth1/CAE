#include "QtMemberWidgetDouble.h"

#include <QDoubleSpinBox>
#include <QVBoxLayout>

QtMemberWidgetDouble::QtMemberWidgetDouble(
        const std::shared_ptr<MemberAccessorInterface<double>>& memberAccessor,
        QWidget* parent,
        double min,
        double max,
        double singleStep,
        int precision)
    : QtMemberWidget<double>(memberAccessor, parent)
{
    mSpinBox = new QDoubleSpinBox();

    QVBoxLayout* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0); // no border

    layout->addWidget(mSpinBox);

    // Connect the check box
    QObject::connect(mSpinBox, SIGNAL(valueChanged(double)),
                     this, SLOT(valueChanged(double)));

    mSpinBox->setRange(min, max);
    mSpinBox->setDecimals(precision);
    mSpinBox->setSingleStep(singleStep);
}

QtMemberWidgetDouble::~QtMemberWidgetDouble()
{

}

void QtMemberWidgetDouble::valueChanged(double value)
{
    mMemberAccessor->setData(value);
}

void QtMemberWidgetDouble::updateSlot()
{
    // Blocking the signals to prevent infinite callback loops.
    mSpinBox->blockSignals(true);
    if (!isAccessorValuesIdentical())
    {
        mSpinBox->clear();
    }
    else
    {
        mSpinBox->setValue(mMemberAccessor->getData());
    }
    mSpinBox->blockSignals(false);
}
