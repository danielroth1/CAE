#include "InterpolatorUIControl.h"
#include "InterpolatorUIForm.h"
#include "ui_InterpolatorUIForm.h"

#include <ui/selection/SelectionControl.h>

#include <scene/data/geometric/MeshInterpolator.h>

InterpolatorUIForm::InterpolatorUIForm(QWidget* parent)
    : QWidget(parent)
    , ui(new Ui::InterpolatorUIForm)
{
    ui->setupUi(this);
}

InterpolatorUIForm::~InterpolatorUIForm()
{
    delete ui;
}

void InterpolatorUIForm::init(InterpolatorUIControl* uiControl,
                              SelectionControl* selectionControl)
{
    mUiControl = uiControl;

    ui->mSelectionSource->init(selectionControl);
    selectionControl->addListener(ui->mSelectionSource);

    ui->mSelectionTarget->init(selectionControl);
    selectionControl->addListener(ui->mSelectionTarget);
}

void InterpolatorUIForm::addInterpolator(SGNode* source, SGNode* target)
{
    std::pair<SGNode*, SGNode*> pair = std::make_pair(source, target);
    auto it = mListWidgetInterpolatorMap.find(pair);
    if (it == mListWidgetInterpolatorMap.end(pair))
    {
        std::string itemString = toString(source, target);
        QListWidgetItem* item = new QListWidgetItem(
                    QString::fromStdString(itemString), ui->mList);
        mListWidgetInterpolatorMap.add(item, pair);
    }
}

void InterpolatorUIForm::removeInterpolator(SGNode* source, SGNode* target)
{
    std::pair<SGNode*, SGNode*> pair = std::make_pair(source, target);
    auto it = mListWidgetInterpolatorMap.find(pair);
    if (it != mListWidgetInterpolatorMap.end(pair))
    {
        QListWidgetItem* item = it->second;

        // remove item from QListWidget
        ui->mList->removeItemWidget(item);
        delete item;

        mListWidgetInterpolatorMap.remove(pair);
    }
}

void InterpolatorUIForm::updateNodeName(SGNode* node, const std::string& /*name*/)
{
    for (const auto& it : mListWidgetInterpolatorMap.getFirstMap())
    {
        QListWidgetItem* item = it.first;
        const std::pair<SGNode*, SGNode*>& pair = it.second;
        if (pair.first == node || pair.second == node)
        {
            item->setText(QString::fromStdString(toString(pair.first, pair.second)));
        }
    }
}

void InterpolatorUIForm::on_mAddButton_clicked()
{
    // Add according to selection source and target
    SGNode* source = ui->mSelectionSource->getSelectedElement();
    SGNode* target = ui->mSelectionTarget->getSelectedElement();

    // Read out type
    MeshInterpolator::Type type;
    if (ui->mFEMMeshRadioButton->isChecked())
    {
        type = MeshInterpolator::Type::FEM;
    }
    else if (ui->mMeshMeshRadioButton->isChecked())
    {
        type = MeshInterpolator::Type::MESH_MESH;
    }
    else
    {
        type = MeshInterpolator::Type::MESH_MESH;
        std::cout << "Unknown interpolation type selected. Mesh-Mesh is used.\n";
    }

    mUiControl->onUiAddInterpolationAction(source, target, type);
}

void InterpolatorUIForm::on_mRemoveButton_clicked()
{
    if (ui->mList->currentItem())
    {
        const std::pair<SGNode*, SGNode*>& p =
                mListWidgetInterpolatorMap.get(ui->mList->currentItem());
        mUiControl->onUiRemoveInterpolationAction(p.first, p.second);
    }
}

std::string InterpolatorUIForm::toString(SGNode* source, SGNode* target)
{
    return source->getName() + " --> " + target->getName();
}

void InterpolatorUIForm::on_mList_itemClicked(QListWidgetItem *item)
{
    std::pair<SGNode*, SGNode*> pair = mListWidgetInterpolatorMap.get(item);
    std::vector<SGNode*> nodes;
    nodes.push_back(pair.first);
    nodes.push_back(pair.second);
    mUiControl->onSelectNodesAction(nodes);
}
