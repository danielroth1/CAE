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

    virtual ~QtMemberWidget() override;

    std::shared_ptr<MemberAccessor<T>> getMemberAccessor() const;

    // AbstractQtMemberWidget interface
public:
    virtual MemberAccessorType getType() const override;
    virtual const std::vector<void*>* getOwners() const override;
    virtual void setOwners(const std::vector<void*>& owners) override;
    virtual void addOwner(void* owner) override;
    virtual void removeOwner(void* owner) override;
    virtual void clearOwners() override;
    virtual bool isAccessorValuesIdentical() override;

protected:    

    std::shared_ptr<MemberAccessor<T>> mMemberAccessor;
};

#include "QtMemberWidget.cpp"

#endif // QTMEMBERWIDGET_H
