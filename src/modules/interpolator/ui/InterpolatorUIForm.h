#ifndef INTERPOLATORUIFORM_H
#define INTERPOLATORUIFORM_H

#include <QWidget>
#include <data_structures/BidirectionalMap.h>
#include <scene/scene_graph/SGCore.h>

class InterpolatorUIControl;
class MeshInterpolator;
class QListWidgetItem;
class SelectionControl;

namespace Ui {
class InterpolatorUIForm;
}

// Provides methods to set info about the currently selected
// vertex.
class InterpolatorUIForm : public QWidget
{
    Q_OBJECT

public:
    InterpolatorUIForm(QWidget *parent = nullptr);
    ~InterpolatorUIForm();

    void init(InterpolatorUIControl* uiControl,
              SelectionControl* selectionControl);

    // Adds interpolator to UI
    void addInterpolator(SGNode* source, SGNode* target);

    // Remove all interpolations
    void removeInterpolator(SGNode* source, SGNode* target);

    // Updates interpolator entries of the given nodes name.
    void updateNodeName(SGNode* node, const std::string& name);

private slots:
    void on_mAddButton_clicked();

    void on_mRemoveButton_clicked();

    void on_mList_itemClicked(QListWidgetItem *item);

private:

    std::string toString(SGNode* source, SGNode* target);

    Ui::InterpolatorUIForm* ui;

    InterpolatorUIControl* mUiControl;

    // list widget <-> interpolator nodes (source, target)
    BidirectionalMap<QListWidgetItem*, std::pair<SGNode*, SGNode*>>
        mListWidgetInterpolatorMap;

};

#endif // INTERPOLATORUIFORM_H
