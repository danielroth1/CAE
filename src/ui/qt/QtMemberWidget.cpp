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

template<class T>
bool QtMemberWidget<T>::hasOwner() const
{
    return mMemberAccessor->hasOwner();
}

template<class T>
void* QtMemberWidget<T>::getOwner()
{
    return mMemberAccessor->getOwner();
}

template<class T>
void QtMemberWidget<T>::setOwner(void* owner)
{
    mMemberAccessor->setOwner(owner);
}

#endif // QTMEMBERWIDGET_CPP
