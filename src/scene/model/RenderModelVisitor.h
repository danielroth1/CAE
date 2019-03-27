#ifndef RENDERMODELVISITOR_H
#define RENDERMODELVISITOR_H

class LinearForceRenderModel;
class Polygon2DTo2DModel;
class Polygon3DTo2DModel;

class RenderModelVisitor
{
public:
    RenderModelVisitor();

    virtual void visit(Polygon2DTo2DModel& model) = 0;

    virtual void visit(Polygon3DTo2DModel& model) = 0;

    virtual void visit(LinearForceRenderModel& model) = 0;

protected:
    virtual ~RenderModelVisitor();
};

#endif // RENDERMODELVISITOR_H
