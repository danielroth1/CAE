#ifndef RENDERMODELVISITOR_H
#define RENDERMODELVISITOR_H

class LinearForceRenderModel;
class PolygonRenderModelImproved;

class RenderModelVisitor
{
public:
    RenderModelVisitor();

    virtual void visit(PolygonRenderModelImproved& model) = 0;

    virtual void visit(LinearForceRenderModel& model) = 0;

protected:
    virtual ~RenderModelVisitor();
};

#endif // RENDERMODELVISITOR_H
