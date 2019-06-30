#ifndef POLYGON2DACCESSOR_H
#define POLYGON2DACCESSOR_H


#include <data_structures/DataStructures.h>


class Polygon2DTopology;

// The Polygon2DAccessor offers an interface to abstract the access to
// Polygon2D and outer meshes of Polygon3D, e.g. in the following example,
// the given poly2 and poly3s outer mesh are both adapted by Polygon2DAccessors.
//
// std::shared_ptr<Polygon2D> poly2 = <some input>;
// std::shared_ptr<Polygon2D> poly3 = <some input>;
// std::vector<std::shared_ptr<Polygon2DAccessor>> accessors;
// accessors.push_back(poly2->createAccessor());
// accessors.push_back(poly3->createAccessor());
//
// for (const std::shared_ptr<Polygon2DAccessor>& a)
//      a->setPosition(0, <some value>);
//
class Polygon2DAccessor
{
public:

    virtual ~Polygon2DAccessor();

    virtual size_t getSize() = 0;

    virtual void setPosition(size_t index, const Eigen::Vector& position) = 0;
    virtual Eigen::Vector& getPosition(std::size_t index) = 0;

    virtual Polygon2DTopology& getTopology2D() = 0;
    virtual const Polygon2DTopology& getTopology2D() const = 0;

    virtual Vectors& getVertexNormals() = 0;

    virtual Vectors& getFaceNormals() = 0;


};

#endif // POLYGON2DACCESSOR_H
