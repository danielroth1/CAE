#include "QtMemberWidgetBool.h"

#include <QCheckBox>
#include <QVBoxLayout>

QtMemberWidgetBool::QtMemberWidgetBool(
        const std::shared_ptr<MemberAccessor<bool>>& memberAccessor,
        QWidget* parent)
    : QtMemberWidget<bool>(memberAccessor, parent)
{
    mCheckBox = new QCheckBox();

    QVBoxLayout* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0); // no border

    layout->addWidget(mCheckBox);

    // Connect the check box
    QObject::connect(mCheckBox, SIGNAL(stateChanged(int)),
                     this, SLOT(checkBoxStateChanged(int)));
}

QtMemberWidgetBool::~QtMemberWidgetBool()
{

}

void QtMemberWidgetBool::checkBoxStateChanged(int state)
{
    mMemberAccessor->setData(state == Qt::CheckState::Checked);
}

void QtMemberWidgetBool::updateSlot()
{
    mCheckBox->blockSignals(true); // Blocking the signals to prevent infinite callback loops.
    mCheckBox->setCheckState(
                mMemberAccessor->getData() ? Qt::CheckState::Checked :
                                             Qt::CheckState::Unchecked);
    mCheckBox->blockSignals(false);
}
