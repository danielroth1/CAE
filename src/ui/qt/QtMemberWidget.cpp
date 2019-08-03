#pragma once

#ifndef QTMEMBERWIDGET_CPP
#define QTMEMBERWIDGET_CPP

#include "QtMemberWidget.h"
#include <iostream>
#include <utils/OwnerMemberAccessor.h>

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
std::shared_ptr<MemberAccessor<T>> QtMemberWidget<T>::getMemberAccessor() const
{
    return mMemberAccessor;
}

template<class T>
MemberAccessorType QtMemberWidget<T>::getType() const
{
    return mMemberAccessor->getType();
}

template<class T>
const std::vector<void*>* QtMemberWidget<T>::getOwners() const
{
    if (mMemberAccessor->getType() == MemberAccessorType::OWNER_MEMBER_ACCESSOR)
    {
        return &(std::static_pointer_cast<OwnerMemberAccessor<T> >(mMemberAccessor)
                ->getOwners());
    }
    else
    {
        std::cout << "Can not set owner of non owner member accessor\n";
        return nullptr;
    }
}

template<class T>
void QtMemberWidget<T>::setOwners(const std::vector<void*>& owners)
{
    if (mMemberAccessor->getType() == MemberAccessorType::OWNER_MEMBER_ACCESSOR)
    {
        std::static_pointer_cast<OwnerMemberAccessor<T> >(mMemberAccessor)
                ->setOwners(owners);
    }
    else
    {
        std::cout << "Can not set owner of non owner member accessor\n";
    }
}

template<class T>
void QtMemberWidget<T>::addOwner(void* owner)
{
    if (mMemberAccessor->getType() == MemberAccessorType::OWNER_MEMBER_ACCESSOR)
    {
        std::static_pointer_cast<OwnerMemberAccessor<T> >(mMemberAccessor)
                ->addOwner(owner);
    }
    else
    {
        std::cout << "Can not add owner of non owner member accessor\n";
    }
}

template<class T>
void QtMemberWidget<T>::removeOwner(void* owner)
{
    if (mMemberAccessor->getType() == MemberAccessorType::OWNER_MEMBER_ACCESSOR)
    {
        std::static_pointer_cast<OwnerMemberAccessor<T> >(mMemberAccessor)
                ->removeOwner(owner);
    }
    else
    {
        std::cout << "Can not remove owner of non owner member accessor\n";
    }
}

template<class T>
void QtMemberWidget<T>::clearOwners()
{
    if (mMemberAccessor->getType() == MemberAccessorType::OWNER_MEMBER_ACCESSOR)
    {
        std::static_pointer_cast<OwnerMemberAccessor<T> >(mMemberAccessor)
                ->clearOwners();
    }
    else
    {
        std::cout << "Can not clear owners of non owner member accessor\n";
    }
}

template<class T>
bool QtMemberWidget<T>::isAccessorValuesIdentical()
{
    if (mMemberAccessor->getType() == MemberAccessorType::OWNER_MEMBER_ACCESSOR)
    {
        return std::static_pointer_cast<OwnerMemberAccessor<T> >(mMemberAccessor)
                ->isAccessorValuesIdentical();
    }
    return true;
}

#endif // QTMEMBERWIDGET_CPP
