#ifndef GEOMETRYINFOUIFORM_H
#define GEOMETRYINFOUIFORM_H

#include <QWidget>
#include <scene/scene_graph/SGCore.h>

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
            QWidget *parent = nullptr);
    ~GeometryInfoUIForm();

    void onSelectedVerticesChanged(
            const std::map<std::shared_ptr<SceneLeafData>, std::vector<ID> >& sv);

    void init();

private slots:
    void on_mSelectedNodesComboBox_currentIndexChanged(int index);

private:

    // Is called if the selected leaf data in the combo box changed.
    // \param index - the index of the selected entry in the combo box
    void onSelectedLeafDataChanged(size_t index);

    Ui::GeometryInfoUIForm* ui;

    std::vector<std::shared_ptr<SceneLeafData>> mSelectedLeafDatasVector;
    std::map<std::shared_ptr<SceneLeafData>, std::vector<ID> > mCurrentSelectionMap;
};

#endif // GEOMETRYINFOUIFORM_H
