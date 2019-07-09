#include "GeometryInfoUIForm.h"
#include "ui_GeometryInfoUIForm.h"

#include <ui/selection/SelectionControl.h>
#include <ui/selection/SelectionListener.h>

GeometryInfoUIForm::GeometryInfoUIForm(
        QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::GeometryInfoUIForm)
{
    ui->setupUi(this);
}

GeometryInfoUIForm::~GeometryInfoUIForm()
{
    delete ui;
}

void GeometryInfoUIForm::onSelectedVerticesChanged(
        const std::map<std::shared_ptr<SceneLeafData>, std::vector<ID> >& sv)
{
    mCurrentSelectionMap = sv;

    mSelectedLeafDatasVector.clear();
    ui->mSelectedNodesComboBox->clear();

    // keep mSelectedLeafDatasVector synchronized and use it to update the ui.
    // mSelectedLeafDatasVector[index] is the leaf data that is behind the
    // indices of the mSelectedNodesComboBox

    // retrieve all selected leaf datas and insert them in a vector
    mSelectedLeafDatasVector.clear();
    for (auto& p : mCurrentSelectionMap)
    {
        mSelectedLeafDatasVector.push_back(p.first);
    }
    for (const std::shared_ptr<SceneLeafData>& ld : mSelectedLeafDatasVector)
    {
        ui->mSelectedNodesComboBox->addItem(
                    QString::fromStdString(ld->getNode()->getName()));
    }

    onSelectedLeafDataChanged(
                static_cast<size_t>(ui->mSelectedNodesComboBox->currentIndex()));
}

void GeometryInfoUIForm::init()
{

}

void GeometryInfoUIForm::on_mSelectedNodesComboBox_currentIndexChanged(int index)
{
    onSelectedLeafDataChanged(static_cast<size_t>(index));
}

void GeometryInfoUIForm::onSelectedLeafDataChanged(size_t index)
{
    ui->mSelectedVertexComboBox->clear();

    if (mSelectedLeafDatasVector.empty())
    {
        return;
    }

    // According to the currently selected leaf data, update the other UI
    // elements.
    std::shared_ptr<SceneLeafData> selectedLeafData =
            mSelectedLeafDatasVector[static_cast<size_t>(index)];

    // fill selected vertices
    auto it = mCurrentSelectionMap.find(selectedLeafData);
    for (ID id : it->second)
    {
        ui->mSelectedVertexComboBox->addItem(QString::number(id));
    }

    // fill neighbored edges
    // differentiate between polygon2dtopology and polygon3dtopology
    // just use Polygon2DAccessor/ Polygon3DAccessor

    // fill neighbored faces
}
