#ifndef ABSTRACTQTMEMBERWIDGET_H
#define ABSTRACTQTMEMBERWIDGET_H


#include <QWidget>


// QtMemberWidget inherit from this class to use QtMemberWidgets without the template parameters.
class AbstractQtMemberWidget : public QWidget
{
    Q_OBJECT

public:

    explicit AbstractQtMemberWidget(QWidget* parent = nullptr);
    virtual ~AbstractQtMemberWidget();

    // Updates the widget with the data value.
    virtual void update();

protected slots:
    // Only call this in the qt thread.
    virtual void updateSlot() = 0;
};

#endif // ABSTRACTQTMEMBERWIDGET_H
