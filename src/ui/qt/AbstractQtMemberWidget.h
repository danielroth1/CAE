#ifndef ABSTRACTQTMEMBERWIDGET_H
#define ABSTRACTQTMEMBERWIDGET_H


#include <QWidget>

#include <utils/MemberAccessor.h>

// QtMemberWidget inherit from this class to use QtMemberWidgets without the template parameters.
class AbstractQtMemberWidget : public QWidget
{
    Q_OBJECT

public:

    explicit AbstractQtMemberWidget(QWidget* parent = nullptr);
    virtual ~AbstractQtMemberWidget();

    // Updates the widget with the data value.
    void update();

    // Delegated OwnerMemberAccessor methods
public:
    virtual MemberAccessorType getType() const = 0;

    // Returns a vector to all owners of this accessor.
    virtual const std::vector<void*>* getOwners() const = 0;

    // Sets the owners of this accessor.
    virtual void setOwners(const std::vector<void*>& owners) = 0;

    virtual void addOwner(void* owner) = 0;
    virtual void removeOwner(void* owner) = 0;
    virtual void clearOwners() = 0;

    virtual bool isAccessorValuesIdentical() = 0;

protected slots:
    // Only call this in the qt thread.
    virtual void updateSlot() = 0;
};

#endif // ABSTRACTQTMEMBERWIDGET_H
