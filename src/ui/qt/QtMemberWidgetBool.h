#ifndef QTMEMBERWIDGETBOOL_H
#define QTMEMBERWIDGETBOOL_H

#include "QtMemberWidget.h"

class QCheckBox;


class QtMemberWidgetBool : public QtMemberWidget<bool>
{
    Q_OBJECT

public:
    explicit QtMemberWidgetBool(
            const std::shared_ptr<MemberAccessor<bool>>& memberAccessor,
            QWidget* parent = nullptr);

    virtual ~QtMemberWidgetBool() override;

private slots:

    void checkBoxStateChanged(int state);

    // Only call this in the qt thread.
    virtual void updateSlot() override;

private:

    QCheckBox* mCheckBox;

};

#endif // QTMEMBERWIDGETBOOL_H
