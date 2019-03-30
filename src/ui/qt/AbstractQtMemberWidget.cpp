#include "AbstractQtMemberWidget.h"

AbstractQtMemberWidget::AbstractQtMemberWidget(QWidget* parent)
    : QWidget(parent)
{

}

AbstractQtMemberWidget::~AbstractQtMemberWidget()
{

}

void AbstractQtMemberWidget::update()
{
    QMetaObject::invokeMethod(this, "updateSlot");
}
