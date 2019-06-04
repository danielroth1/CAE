#include "SGUIControl.h"

#include <ui/UIControl.h>

#include <scene/scene_graph/SGControl.h>
#include <scene/scene_graph/SGCore.h>

#include <scene/model/PolygonRenderModel.h>
#include <scene/model/RenderModel.h>
#include <scene/model/RenderModelVisitor.h>

SGUIControl::SGUIControl()
    : mVisualizeFaceNormals(false)
    , mVisualizeVertexNormals(false)
{

}

void SGUIControl::initialize(SGControl* sgControl, UIControl* uiControl)
{
    mSgControl = sgControl;
    mUiControl = uiControl;
}

void SGUIControl::setVisualizeFaceNormals(bool visualizeFaceNormals)
{
    class NormalVisualizerRenderModel : public RenderModelVisitor
    {
    public:
        NormalVisualizerRenderModel(bool _renderFaceNormals)
            : renderFaceNormals(_renderFaceNormals)
        {

        }
        virtual void visit(PolygonRenderModel& model)
        {
            model.setRenderFaceNormals(renderFaceNormals);
        }
        virtual void visit(LinearForceRenderModel& /*model*/)
        {

        }
        bool renderFaceNormals;
    } renderModelVisitor(visualizeFaceNormals);

    iterateRenderModels(renderModelVisitor);
}

bool SGUIControl::isVisualizeFaceNormals() const
{
    return mVisualizeFaceNormals;
}

void SGUIControl::setVisualizeVertexNormals(bool visualizeVertexNormals)
{
    class NormalVisualizerRenderModel : public RenderModelVisitor
    {
    public:
        NormalVisualizerRenderModel(bool _renderVertexNormals)
            : renderVertexNormals(_renderVertexNormals)
        {

        }
        virtual void visit(PolygonRenderModel& model)
        {
            model.setRenderVertexNormals(renderVertexNormals);
        }
        virtual void visit(LinearForceRenderModel& /*model*/)
        {

        }
        bool renderVertexNormals;
    } renderModelVisitor(visualizeVertexNormals);

    iterateRenderModels(renderModelVisitor);
}

bool SGUIControl::isVisualizeVertexNormals() const
{
    return mVisualizeVertexNormals;
}

void SGUIControl::iterateRenderModels(RenderModelVisitor& renderModelVisitor)
{
    SGTraverser tt(mSgControl->getSceneGraph()->getRoot());
    class RevalidateVisitor : public SGNodeVisitor
    {
    public:
        RevalidateVisitor(RenderModelVisitor& _renderModelVisitor)
            : renderModelVisitor(_renderModelVisitor)
        {
        }
        virtual void visit(SGChildrenNode* /*childrenNode*/)
        {
        }
        virtual void visit(SGLeafNode* leafNode)
        {
            std::shared_ptr<RenderModel> rm = leafNode->getData()->getRenderModel();
            if (rm)
                rm->accept(renderModelVisitor);
        }

        RenderModelVisitor& renderModelVisitor;
    } visitor(renderModelVisitor);

    tt.traverse(visitor);
}
