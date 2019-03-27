#ifndef GEOMETRICPOINTREFVISITOR_H
#define GEOMETRICPOINTREFVISITOR_H

class GeometricVertexRef;
class PolygonVectorRef;

class GeometricPointRefVisitor
{
public:
    virtual void visit(GeometricVertexRef& ref) = 0;
    virtual void visit(PolygonVectorRef& ref) = 0;

protected:
    GeometricPointRefVisitor();
    virtual ~GeometricPointRefVisitor();
};

#endif // GEOMETRICPOINTREFVISITOR_H
