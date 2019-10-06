#ifndef VERTEXREF_H
#define VERTEXREF_H

#include "GeometricPointRef.h"

class GeometricVertexRef : public GeometricPointRef
{
public:
    GeometricVertexRef(GeometricData* geometricData, ID index);

    // GeometricPointRef interface
public:

    virtual Vector getPoint() const override;

    virtual ID getIndex() const override;

    virtual GeometricPointRef* clone() override;

    virtual void accept(GeometricPointRefVisitor& visitor) override;

private:

    ID mIndex;
};

#endif // VERTEXREF_H
