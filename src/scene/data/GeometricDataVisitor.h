#ifndef GEOMETRICDATAVISITOR_H
#define GEOMETRICDATAVISITOR_H

class GeometricPoint;
class Polygon2D;
class Polygon3D;

class GeometricDataVisitor
{
public:
    virtual ~GeometricDataVisitor();

    virtual void visit(Polygon2D& polygon2D) = 0;
    virtual void visit(Polygon3D& polygon3D) = 0;
    virtual void visit(GeometricPoint& point) = 0;

protected:
    GeometricDataVisitor();
};

#endif // GEOMETRICDATAVISITOR_H
