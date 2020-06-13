#ifndef GEOMETRYINFOUIFORM_H
#define GEOMETRYINFOUIFORM_H

#include <QWidget>
#include <scene/scene_graph/SGCore.h>

class SelectionControl;

namespace Ui {
class GeometryInfoUIForm;
}

// Provides methods to set info about the currently selected
// vertex.
class GeometryInfoUIForm : public QWidget
{
    Q_OBJECT

public:
    explicit GeometryInfoUIForm(
            SelectionControl* sc,
            QWidget *parent = nullptr);
    ~GeometryInfoUIForm();

    void onSelectedVerticesChanged(
            const std::map<std::shared_ptr<SceneLeafData>, std::vector<ID> >& sv);

    void init();

private slots:
    void on_mSelectedNodesComboBox_currentIndexChanged(int index);

    // Is called if the combo box of the vertex ids changed. Updates
    // neighbores edges / faces.
    void on_mSelectedVertexComboBox_currentIndexChanged(int index);

    // Is called when the "Select Vertex" button is clicked. Selects the
    // vertex with the id that is selected in the above combo box. If the
    // id is too large, the vertex with the highest id is selected.
    void on_mSelectVertexButton_clicked();

private:

    // Is called if the selected leaf data in the combo box changed.
    // \param index - the index of the selected entry in the combo box
    void onSelectedLeafDataChanged(size_t index);

    Ui::GeometryInfoUIForm* ui;

    SelectionControl* mSc;

    std::vector<std::shared_ptr<SceneLeafData>> mSelectedLeafDatasVector;
    std::map<std::shared_ptr<SceneLeafData>, std::vector<ID> > mCurrentSelectionMap;
};

#endif // GEOMETRYINFOUIFORM_H
