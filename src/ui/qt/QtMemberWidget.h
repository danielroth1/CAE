#ifndef QTMEMBERWIDGET_H
#define QTMEMBERWIDGET_H

#include "AbstractQtMemberWidget.h"

#include <utils/MemberAccessor.h>

#include <memory>
#include <QWidget>



template <class T>
class QtMemberWidget : public AbstractQtMemberWidget
{
public:

    QtMemberWidget(
            const std::shared_ptr<MemberAccessor<T>>& memberAccessor,
            QWidget* parent = nullptr);

    virtual ~QtMemberWidget();

protected:    

    std::shared_ptr<MemberAccessor<T>> mMemberAccessor;
};

#include "QtMemberWidget.cpp"

#endif // QTMEMBERWIDGET_H