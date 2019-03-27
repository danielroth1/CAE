#include "MeshConverterUIControl.h"
#include "MeshConverterUIForm.h"
#include "ui_MeshConverterUIForm.h"

MeshConverterUIForm::MeshConverterUIForm(MeshConverterUIControl* uiControl,
                                         QWidget *parent) :
    QWidget(parent),
    ui(new Ui::MeshConverterUIForm)
{
    ui->setupUi(this);
    mUiControl = uiControl;
}

MeshConverterUIForm::~MeshConverterUIForm()
{
    delete ui;
}

double MeshConverterUIForm::getCellSize()
{
    return ui->mCellSizeBox->value();
}

double MeshConverterUIForm::getCellRadiusEdgeRatio()
{
    return ui->mCellRadiusEdgeRatioBox->value();
}

void MeshConverterUIForm::updateCellSize(double cellSize)
{
    updateDoubleSpinBox(ui->mCellSizeBox, cellSize);
}

void MeshConverterUIForm::updateCellRadiusEdgeRatio(double cellRadiusEdgeRatio)
{
    updateDoubleSpinBox(ui->mCellRadiusEdgeRatioBox, cellRadiusEdgeRatio);
}

void MeshConverterUIForm::updateGeometricData(int nVertices, int nTriangles, int nTetrahedrons)
{
    updateNumVertices(nVertices);
    updateNumTriangles(nTriangles);
    updateNumTetrahedrons(nTetrahedrons);
}

void MeshConverterUIForm::updateNumVertices(int nVertices)
{
    ui->mNVerticesEdit->blockSignals(true);
    ui->mNVerticesEdit->setText(QString::number(nVertices));
    ui->mNVerticesEdit->blockSignals(false);
}

void MeshConverterUIForm::updateNumTriangles(int nTriangles)
{
    ui->mNTrianglesEdit->blockSignals(true);
    ui->mNTrianglesEdit->setText(QString::number(nTriangles));
    ui->mNTrianglesEdit->blockSignals(false);
}

void MeshConverterUIForm::updateNumTetrahedrons(int nTetrahedrons)
{
    ui->mNTetrahedronsEdit->blockSignals(true);
    ui->mNTetrahedronsEdit->setText(QString::number(nTetrahedrons));
    ui->mNTetrahedronsEdit->blockSignals(false);
}

void MeshConverterUIForm::on_mConvertButton_clicked()
{
    mUiControl->onConvertButtonClicked();
}

void MeshConverterUIForm::on_mRevertButton_clicked()
{
    mUiControl->onRevertButtonClicked();
}

void MeshConverterUIForm::updateDoubleSpinBox(
        QDoubleSpinBox* spinBox, double value)
{
    spinBox->blockSignals(true);
    spinBox->setValue(value);
    spinBox->blockSignals(false);
}

void MeshConverterUIForm::updateLineEdget(QLineEdit* lineEdit, double value)
{
    lineEdit->blockSignals(true);
    lineEdit->setText(QString::number(value)); // convert double to string
    lineEdit->blockSignals(false);
}
