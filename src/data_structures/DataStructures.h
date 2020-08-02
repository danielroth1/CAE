#ifndef DATASTRUCTURES_H
#define DATASTRUCTURES_H

#include <array>
#include <memory>
#include <vector>

#include <Eigen/Dense>

// Makes the code compatible to older eigen version 3.2.10
namespace Eigen {
    typedef EIGEN_DEFAULT_DENSE_INDEX_TYPE Index;
}

namespace Eigen
{
    typedef Vector3d Vector;
    typedef Vector3f Vectorf;
}

typedef std::array<unsigned int, 2> Edge;
typedef std::array<unsigned int, 3> Face;
typedef std::array<unsigned int, 4> Cell;

typedef std::vector<Eigen::Vector> Vectors;
typedef std::vector<Eigen::Vectorf> Vectorfs;
typedef std::vector<unsigned int> VertexIds;
typedef std::vector<Edge> Edges;
typedef std::vector<Face> Faces;
typedef std::vector<Cell> Cells;

typedef std::vector<std::array<unsigned int, 2>> Lines;

typedef size_t ID;

extern ID ILLEGAL_INDEX;

#endif // DATASTRUCTURES_H
