#ifndef SELECTIONSCENEDATAMODEL_H
#define SELECTIONSCENEDATAMODEL_H

#include <data_structures/DataStructures.h>
#include <scene/model/RenderModel.h>
#include <memory>

class RenderPoints;
class SelectionSceneData;

class SelectionSceneDataModel : public RenderModel
{
public:
    SelectionSceneDataModel(SelectionSceneData& selectionSceneData);

    virtual ~SelectionSceneDataModel() override;

    // RenderModel interface
public:
    virtual void reset() override;
    virtual void update() override;
    virtual void revalidate() override;
    virtual void accept(RenderModelVisitor& v) override;
    virtual void addToRenderer(Renderer* renderer) override;
    virtual void removeFromRenderer(Renderer* renderer) override;
    virtual void setVisible(bool visible) override;

private:
    SelectionSceneData& mSelectionSceneData;

    std::shared_ptr<RenderPoints> mRenderPoints;
};

#endif // SELECTIONSCENEDATAMODEL_H
