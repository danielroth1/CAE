#ifndef RENDEROBJECTVISITOR_H
#define RENDEROBJECTVISITOR_H

class RenderLine;
class RenderPolygon2D;
class RenderPoints;
class RenderScreenRectangle;

class RenderObjectVisitor
{
public:
    RenderObjectVisitor();
    virtual ~RenderObjectVisitor();

    virtual void visit(RenderPolygon2D& rp) = 0;

    virtual void visit(RenderPoints& rp) = 0;

    virtual void visit(RenderScreenRectangle&) = 0;

    virtual void visit(RenderLine&) = 0;

};

#endif // RENDEROBJECTVISITOR_H
