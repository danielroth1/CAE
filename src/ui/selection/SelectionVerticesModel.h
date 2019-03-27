#ifndef SELECTIONMODEL_H
#define SELECTIONMODEL_H

// Includes
#include <data_structures/DataStructures.h>
#include <scene/model/RenderModel.h>
#include <set>
#include <ui/UniqueVertex.h>

// Forward Declarations
class Renderer;
class RenderScreenRectangle;
class RenderPoints;
class SelectionVertices;
class ViewFrustum;

class SelectionVerticesModel : public RenderModel
{
public:
    SelectionVerticesModel(
            SelectionVertices& selection);

   virtual ~SelectionVerticesModel() override;

    // Transforms given points to their corresponding floating
    // point representation. Call this method each time the point
    // set changes. This is the case if
    // - a point was added
    // - a point was removed
    // - a point changed its value
    void updatePoints();

    // Informs the renderer about the change of the four screen
    // rectangle coordinates. Call this function if one of the
    // coordinates changed.
    void updateScreenRectangle();

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

    std::shared_ptr<RenderPoints> mRenderPoints;

    SelectionVertices& mSelection;

};

#endif // SELECTIONMODEL_H
