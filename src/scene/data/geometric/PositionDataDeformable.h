#ifndef GEOMETRICDATADEFORMABLE_H
#define GEOMETRICDATADEFORMABLE_H

#include <scene/data/GeometricData.h>



class GeometricDataDeformable : public GeometricData
{
public:
    GeometricDataDeformable(const Vectors& positions);

    // GeometricData method
    // Returns position at the given index.
    virtual Vector& getPosition(size_t index) override;

    // GeometricData method
    // Returns number of positions.
    virtual size_t getSize() override;

private:
    Vectors mPositions;
};

#endif // GEOMETRICDATADEFORMABLE_H
