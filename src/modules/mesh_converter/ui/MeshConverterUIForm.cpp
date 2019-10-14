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
    if (ui->mCellSizeCheckBox->checkState() == Qt::Checked)
        return ui->mCellSizeBox->value();
    else
        return 0.0;
}

double MeshConverterUIForm::getCellRadiusEdgeRatio()
{
    if (ui->mCellRadiusEdgeRatioCheckBox->checkState() == Qt::Checked)
        return ui->mCellRadiusEdgeRatioBox->value();
    else
        return 0.0;
}

double MeshConverterUIForm::getFacetAngle() const
{
    if (ui->mFacetAngleCheckBox->checkState() == Qt::Checked)
        return ui->mFacetAngle->value();
    else
        return 0.0;
}

double MeshConverterUIForm::getFacetSize() const
{
    if (ui->mFacetSizeCheckBox->checkState() == Qt::Checked)
        return ui->mFacetSize->value();
    else
        return 0.0;
}

double MeshConverterUIForm::getFacetDistance() const
{
    if (ui->mFacetDistanceCheckBox->checkState() == Qt::Checked)
        return ui->mFacetDistance->value();
    else
        return 0.0;
}

bool MeshConverterUIForm::isSharpFeaturesEnabled() const
{
    return ui->mSharpFeaturesEnabled->checkState() == Qt::Checked;
}

double MeshConverterUIForm::getMinFeatureEdgeAngleDeg() const
{
    return ui->mMinFeatureEdgeAngleDeg->value();
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

void MeshConverterUIForm::on_mCellSizeCheckBox_stateChanged(int value)
{
    ui->mCellSizeBox->setEnabled(value == Qt::Checked);
}

void MeshConverterUIForm::on_mCellRadiusEdgeRatioBox_2_stateChanged(int value)
{
    ui->mCellRadiusEdgeRatioBox->setEnabled(value == Qt::Checked);
}

void MeshConverterUIForm::on_mFacetAngleCheckBox_stateChanged(int value)
{
    ui->mFacetAngle->setEnabled(value == Qt::Checked);
}

void MeshConverterUIForm::on_mFacetSizeCheckBox_stateChanged(int value)
{
    ui->mFacetSize->setEnabled(value == Qt::Checked);
}

void MeshConverterUIForm::on_mFacetDistanceCheckBox_stateChanged(int value)
{
    ui->mFacetDistance->setEnabled(value == Qt::Checked);
}

void MeshConverterUIForm::on_mSharpFeaturesEnabled_stateChanged(int value)
{
    ui->mMinFeatureEdgeAngleDeg->setEnabled(value == Qt::Checked);
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
