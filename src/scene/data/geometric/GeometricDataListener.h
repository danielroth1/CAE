#ifndef GEOMETRICDATALISTENER_H
#define GEOMETRICDATALISTENER_H


class GeometricDataListener
{
public:
    GeometricDataListener();
    virtual ~GeometricDataListener();

    // Is called when the listened geometric data changed.
    virtual void notifyGeometricDataChanged() = 0;
};

#endif // GEOMETRICDATALISTENER_H
