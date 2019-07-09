#ifndef GEOMETRYINFOUICONTROL_H
#define GEOMETRYINFOUICONTROL_H

#include <ui/selection/SelectionListener.h>

class GeometryInfoUIForm;
class QWidget;
class SelectionControl;

class GeometryInfoUIControl : public SelectionListener
{
public:
    GeometryInfoUIControl(SelectionControl* mSc);


    // Adds itself as listener to SelectionControl.
    void init(QWidget* widget);

    // Removes itself as listener from SelectionControl.
    void finalize();

    QWidget* getWidget();

private:
    SelectionControl* mSc;
    GeometryInfoUIForm* mForm;

    // SelectionListener interface
public:
    virtual void onSceneNodeSelected(const std::shared_ptr<SceneData>& sd);

    virtual void onSelectedSceneNodesChanged(
            const std::set<std::shared_ptr<SceneData> >& sd);

    virtual void onSelectedVerticesChanged(
            const std::map<std::shared_ptr<SceneLeafData>, std::vector<ID> >& sv);
};

#endif // GEOMETR>INFOUICONTROL_H
