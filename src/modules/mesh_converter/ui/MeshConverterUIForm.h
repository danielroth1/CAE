#ifndef MESHCONVERTERUIFORM_H
#define MESHCONVERTERUIFORM_H

#include <QWidget>

class MeshConverterUIControl;
class QDoubleSpinBox;
class QLineEdit;

namespace Ui {
class MeshConverterUIForm;
}

class MeshConverterUIForm : public QWidget
{
    Q_OBJECT

public:
    explicit MeshConverterUIForm(
            MeshConverterUIControl* mUiControl,
            QWidget *parent = nullptr);
    ~MeshConverterUIForm();

    // Getters for UI elements
        double getCellSize();
        double getCellRadiusEdgeRatio();
        double getFacetAngle() const;
        double getFacetSize() const;
        double getFacetDistance() const;

    // Methods for updatingUI
    // Signals are blocked while updating, preventing infite loops.
        void updateCellSize(double cellSize);
        void updateCellRadiusEdgeRatio(double cellRadiusEdgeRatio);
        void updateGeometricData(int nVertices,
                                 int nTriangles,
                                 int nTetrahedrons);
        void updateNumVertices(int nVertices);
        void updateNumTriangles(int nTriangles);
        void updateNumTetrahedrons(int nTetrahedrons);

private slots:

    void on_mConvertButton_clicked();

    void on_mRevertButton_clicked();

private:

    // Update a double spin box with a given value.
    // Signals are blocked while updating.
    void updateDoubleSpinBox(QDoubleSpinBox* spinBox, double value);
    void updateLineEdget(QLineEdit* lineEdit, double value);

    MeshConverterUIControl* mUiControl;

    Ui::MeshConverterUIForm *ui;
};

#endif // MESHCONVERTERUIFORM_H
