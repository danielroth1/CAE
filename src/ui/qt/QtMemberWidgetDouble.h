#ifndef QTMEMBERWIDGETDOUBLE_H
#define QTMEMBERWIDGETDOUBLE_H

#include "QtMemberWidget.h"


class QDoubleSpinBox;

class QtMemberWidgetDouble : public QtMemberWidget<double>
{
    Q_OBJECT

public:

    explicit QtMemberWidgetDouble(
            const std::shared_ptr<MemberAccessorInterface<double>>& memberAccessor,
            QWidget* parent = nullptr,
            double min = 0.0,
            double max = 100.0,
            double singleStep = 1.0,
            int precision = 3);

    virtual ~QtMemberWidgetDouble() override;

private slots:

    void valueChanged(double value);

    // Only call this in the qt thread.
    virtual void updateSlot() override;

private:

    QDoubleSpinBox* mSpinBox;

};

#endif // QTMEMBERWIDGETDOUBLE_H
