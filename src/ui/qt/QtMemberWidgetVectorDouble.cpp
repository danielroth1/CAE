#include "QtMemberWidgetVectorDouble.h"

#include <QDoubleSpinBox>
#include <QVBoxLayout>

QtMemberWidgetVectorDouble::QtMemberWidgetVectorDouble(
        const std::shared_ptr<MemberAccessorInterface<Eigen::Vector3d>>& memberAccessor,
        QWidget* parent,
        double min,
        double max,
        double singleStep,
        int precision)
    : QtMemberWidget<Eigen::Vector3d>(memberAccessor, parent)
{
    QVBoxLayout* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0); // no border

    mSpinBoxWidget = new QWidget();
    mSpinBoxWidget->setLayout(new QVBoxLayout());
    mSpinBoxWidget->layout()->setContentsMargins(0, 0, 0, 0);

    for (size_t i = 0; i < 3; ++i)
    {
        QDoubleSpinBox* spinBox = createSpinBox(min, max, singleStep, precision);
        mSpinBoxes.push_back(spinBox);
        mSpinBoxWidget->layout()->addWidget(spinBox);
    }

    layout->addWidget(mSpinBoxWidget);
}

QtMemberWidgetVectorDouble::~QtMemberWidgetVectorDouble()
{

}

void QtMemberWidgetVectorDouble::valueChanged(double /*value*/)
{
    setMemberAccessorDataFromUI();
}

void QtMemberWidgetVectorDouble::updateSlot()
{
    // Blocking the signals to prevent infinite callback loops.
    for (size_t i = 0; i < mSpinBoxes.size(); ++i)
    {
        QDoubleSpinBox* spinBox = mSpinBoxes[i];

        spinBox->blockSignals(true);

        if (!isAccessorValuesIdentical())
        {
            spinBox->clear();
        }
        else
        {
            spinBox->setValue(mMemberAccessor->getData()(i));
        }
        spinBox->blockSignals(false);
    }
}

QDoubleSpinBox* QtMemberWidgetVectorDouble::createSpinBox(
        double min,
        double max,
        double singleStep,
        int precision)
{
    QDoubleSpinBox* spinBox = new QDoubleSpinBox();

    // Connect the check box
    QObject::connect(spinBox, SIGNAL(valueChanged(double)),
                     this, SLOT(valueChanged(double)));

    spinBox->setRange(min, max);
    spinBox->setDecimals(precision);
    spinBox->setSingleStep(singleStep);

    mSpinBoxWidget->layout()->addWidget(spinBox);
    return spinBox;
}

void QtMemberWidgetVectorDouble::setMemberAccessorDataFromUI()
{
    Eigen::Vector3d value;
    for (size_t i = 0; i < 3; ++i)
    {
        value[i] = mSpinBoxes[i]->value();
    }
    mMemberAccessor->setData(value);
}
