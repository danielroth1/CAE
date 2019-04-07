#ifndef QTMEMBERWIDGETVECTORDOUBLE_H
#define QTMEMBERWIDGETVECTORDOUBLE_H

#include "QtMemberWidget.h"

#include <Eigen/Dense>


class QDoubleSpinBox;

class QtMemberWidgetVectorDouble : public QtMemberWidget<Eigen::Vector3d>
{
    Q_OBJECT

public:

    explicit QtMemberWidgetVectorDouble(
            const std::shared_ptr<MemberAccessor<Eigen::Vector3d>>& memberAccessor,
            QWidget* parent = nullptr,
            double min = 0.0,
            double max = 1000.0,
            double singleStep = 1.0,
            int precision = 3);

    virtual ~QtMemberWidgetVectorDouble() override;

private slots:

    void valueChanged(double value);

    // Only call this in the qt thread.
    virtual void updateSlot() override;

private:

    QDoubleSpinBox* createSpinBox(double min,
                                  double max,
                                  double singleStep,
                                  int precision);

    void setMemberAccessorDataFromUI();

    // Widget that containts the spin boxes
    QWidget* mSpinBoxWidget;

    std::vector<QDoubleSpinBox*> mSpinBoxes;

};

#endif // QTMEMBERWIDGETVECTORDOUBLE_H
