#ifndef MESHCONVERTERUICONTROL_H
#define MESHCONVERTERUICONTROL_H

#include <data_structures/DataStructures.h>
#include <set>
#include <map>
#include <ui/selection/SelectionListener.h>

class ApplicationControl;
class GeometricData;
class MeshConverterModule;
class MeshConverterUIForm;
class QWidget;
class SceneData;
class SceneLeafData;

// This class has two tasks:
//  - update UI after changes:
//      -> Manly, after a model is selected, update (!)
//          #vertices, #triangles, and #tetrahedrons.
//  - implement functionality of buttons
class MeshConverterUIControl : public SelectionListener
{
public:
    MeshConverterUIControl(MeshConverterModule* module,
                           ApplicationControl* ac);

    virtual ~MeshConverterUIControl();

    void init(QWidget* parent);

    QWidget* getWidget();

    void onConvertButtonClicked();
    void onRevertButtonClicked();

    // Updates ui with the given geometry.
    void updateUIFromGeometry(GeometricData* geometricData);

    // SelectionListener interface
public:
    virtual void onSceneNodeSelected(SceneData* sd);
    virtual void onSelectedSceneNodesChanged(
            const std::set<SceneData*>& sd);
    virtual void onSelectedVerticesChanged(
            const std::map<SceneLeafData*, std::vector<ID> >& sv);

private:

    void onSelectionCleared();
    void processSceneData(SceneData* sd);
    void processGeometricData(GeometricData* gd);

    ApplicationControl* mAc;
    MeshConverterModule* mModule;
    MeshConverterUIForm* mWidget;
};

#endif // MESHCONVERTERUICONTROL_H
