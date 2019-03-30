#pragma once

#ifndef QTMEMBERWIDGET_CPP
#define QTMEMBERWIDGET_CPP

#include "QtMemberWidget.h"

template<class T>
QtMemberWidget<T>::QtMemberWidget(
        const std::shared_ptr<MemberAccessor<T>>& memberAccessor,
        QWidget* parent)
    : AbstractQtMemberWidget(parent)
    , mMemberAccessor(memberAccessor)
{

}

template<class T>
QtMemberWidget<T>::~QtMemberWidget()
{

}

#endif // QTMEMBERWIDGET_CPP
