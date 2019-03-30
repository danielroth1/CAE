#ifndef QTMEMBERWIDGETDOUBLE_H
#define QTMEMBERWIDGETDOUBLE_H

#include "QtMemberWidget.h"


class QDoubleSpinBox;

class QtMemberWidgetDouble : public QtMemberWidget<double>
{
    Q_OBJECT

public:

    explicit QtMemberWidgetDouble(
            const std::shared_ptr<MemberAccessor<double>>& memberAccessor,
            QWidget* parent = nullptr);

    virtual ~QtMemberWidgetDouble() override;

private slots:

    void valueChanged(double value);

    // Only call this in the qt thread.
    virtual void updateSlot() override;

private:

    QDoubleSpinBox* mSpinBox;

};

#endif // QTMEMBERWIDGETDOUBLE_H
