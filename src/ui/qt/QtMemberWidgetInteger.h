#ifndef QTMEMBERWIDGETINTEGER_H
#define QTMEMBERWIDGETINTEGER_H

#include "QtMemberWidget.h"


class QSpinBox;

class QtMemberWidgetInteger : public QtMemberWidget<int>
{
    Q_OBJECT

public:

    explicit QtMemberWidgetInteger(
            const std::shared_ptr<MemberAccessorInterface<int>>& memberAccessor,
            QWidget* parent = nullptr,
            int min = 0,
            int max = 100,
            int singleStep = 1);

    virtual ~QtMemberWidgetInteger() override;

private slots:

    void valueChanged(int value);

    // Only call this in the qt thread.
    virtual void updateSlot() override;

private:

    QSpinBox* mSpinBox;

};

#endif // QTMEMBERWIDGETINTEGER_H
