#include "GeometryInfoUIForm.h"
#include "ui_GeometryInfoUIForm.h"

#include <ui/selection/SelectionControl.h>
#include <ui/selection/SelectionListener.h>

#include <scene/data/GeometricData.h>
#include <scene/data/GeometricDataVisitor.h>

#include <scene/data/geometric/Polygon2D.h>
#include <scene/data/geometric/Polygon3D.h>
#include <scene/data/geometric/PolygonTopology.h>

#include <scene/VertexCollection.h>

GeometryInfoUIForm::GeometryInfoUIForm(
        SelectionControl* sc,
        QWidget* parent)
    : QWidget(parent)
    , ui(new Ui::GeometryInfoUIForm)
    , mSc(sc)
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
}

void GeometryInfoUIForm::on_mSelectedVertexComboBox_currentIndexChanged(int index)
{
    // fill neighbored edges
    // differentiate between polygon2dtopology and polygon3dtopology
    // just use Polygon2DAccessor/ Polygon3DAccessor

    bool ok;
    int id = ui->mSelectedVertexComboBox->itemText(index).toInt(&ok);

    if (!ok)
    {
        std::cout << "Vertex combo box does not contain number. Should not"
                     " be happening.\n";
        return;
    }

    // fill neighbored edges and faces
    class GeometricInfoExtractor : public GeometricDataVisitor
    {
    public:
        GeometricInfoExtractor(ID vertexId)
            : mVertexId(vertexId)
        {

        }

        void visitPolygon(AbstractPolygon& polygon)
        {
            for (ID id : polygon.getTopology().getVertex(mVertexId).getEdgeIds())
            {
                mEdgeIds.push_back(id);
            }

            for (ID id : polygon.getTopology().getVertex(mVertexId).getFaceIds())
            {
                mFaceIds.push_back(id);
            }
        }

        virtual void visit(Polygon2D& polygon2D)
        {
            visitPolygon(polygon2D);
        }

        virtual void visit(Polygon3D& polygon3D)
        {
            visitPolygon(polygon3D);
        }

        virtual void visit(GeometricPoint& /*point*/)
        {
            // Point has no neighbored edges / faces.
        }

        ID mVertexId;
        std::vector<ID> mFaceIds;
        std::vector<ID> mEdgeIds;
    } visitor(id);

    std::shared_ptr<SceneLeafData> selectedLeafData =
            mSelectedLeafDatasVector[static_cast<size_t>(ui->mSelectedNodesComboBox->currentIndex())];
    selectedLeafData->getGeometricData()->accept(visitor);

    // First, remove old values from combo box.
    ui->mNeighboredEdgesComboBox->clear();
    ui->mNeighboredFacesComboBox->clear();

    // Add new values to combo box.
    for (ID idEdge : visitor.mEdgeIds)
    {
        ui->mNeighboredEdgesComboBox->addItem(QString::number(idEdge));
    }

    for (ID idFace : visitor.mFaceIds)
    {
        ui->mNeighboredFacesComboBox->addItem(QString::number(idFace));
    }
}

void GeometryInfoUIForm::on_mSelectVertexButton_clicked()
{
    std::shared_ptr<SceneLeafData> selectedLeafData =
            mSelectedLeafDatasVector[static_cast<size_t>(ui->mSelectedNodesComboBox->currentIndex())];

    VertexCollection vc;

    int vertexId = ui->mSelectVertexSpinBox->value();
    std::shared_ptr<AbstractPolygon> poly =
            std::dynamic_pointer_cast<AbstractPolygon>(selectedLeafData->getGeometricData());
    if (poly)
    {
        vc.addVertex(selectedLeafData,
                     std::min(poly->getSize() - 1, static_cast<size_t>(vertexId)));
        mSc->setVertexSelection(vc);
    }
    else
    {
        std::cout << "Cannot select non polygon vertices for the moment.\n";
    }

}
