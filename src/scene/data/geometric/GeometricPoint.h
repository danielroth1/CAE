#ifndef GEOMETRICPOINT_H
#define GEOMETRICPOINT_H

#include <scene/data/GeometricData.h>



class GeometricPoint : public GeometricData
{
public:
    GeometricPoint(Vector position);

    Vector& getPosition();
    void setPosition(const Eigen::Vector& position);

    // GeometricData interface
public:
    Type getType() const override;
    void updateBoundingBox() override;
    void accept(GeometricDataVisitor& visitor) override;
    Vector& getPosition(size_t index) override;
    size_t getSize() override;
    void translate(const Eigen::Vector& position) override;
    void transform(const Eigen::Affine3d& transform) override;

private:
    Vector mPosition;

};

#endif // GEOMETRICPOINT_H
