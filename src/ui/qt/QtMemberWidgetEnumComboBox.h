#ifndef QTMEMBERWIDGETENUMCOMBOBOX_H
#define QTMEMBERWIDGETENUMCOMBOBOX_H

#include "QtMemberWidget.h"

#include <Eigen/Dense>


class QComboBox;

// A combo box that can be used to set enum values.
class QtMemberWidgetEnumComboBox : public QtMemberWidget<int>
{
    Q_OBJECT

public:

    explicit QtMemberWidgetEnumComboBox(
            const std::shared_ptr<MemberAccessorInterface<int>>& memberAccessor,
            QWidget* parent = nullptr,
            const std::vector<std::string>& enumNames = std::vector<std::string>());

    virtual ~QtMemberWidgetEnumComboBox() override;

private slots:

    void currentIndexChanged(int value);

    // Only call this in the qt thread.
    virtual void updateSlot() override;

private:

    QComboBox* createComboBox(const std::vector<std::string>& enumNames);

    QComboBox* mComboBox;

};

#endif // QTMEMBERWIDGETENUMCOMBOBOX_H
