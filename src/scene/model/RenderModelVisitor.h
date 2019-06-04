#ifndef RENDERMODELVISITOR_H
#define RENDERMODELVISITOR_H

class LinearForceRenderModel;
class PolygonRenderModel;

class RenderModelVisitor
{
public:
    RenderModelVisitor();

    virtual void visit(PolygonRenderModel& model) = 0;

    virtual void visit(LinearForceRenderModel& model) = 0;

protected:
    virtual ~RenderModelVisitor();
};

#endif // RENDERMODELVISITOR_H
