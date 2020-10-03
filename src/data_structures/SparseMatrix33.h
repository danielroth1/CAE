#ifndef SPARSEMATRIX33_H
#define SPARSEMATRIX33_H

#include <array>
#include <map>
#include <Eigen/Sparse>
#include <vector>

// Makes the code compatible to older eigen version 3.2.10
namespace Eigen {
typedef EIGEN_DEFAULT_DENSE_INDEX_TYPE Index;
}

// Represents a stiffness matrix that is composed of 3x3 sub matrices.
// Offers efficient methods to access these sub matrices with three block
// operations for each submatrix that accesses the three rows of the matrix.
// Internally stores a regular sparse matrix consisting of doubles that
// can be used for regular calculations.
//
// Allows for efficient access
// Sparse matrices can be constructed once. Then the memory can be accessed
// directly as long as no more element is inserted (only if the sparsity pattern
// would change).
//
// How to use:
// Construction:
//      Before using the matrix, its sparsity pattern must be assigned (It must
//      be defined where there are non-zero entries in the matrix). The actual
//      values of these entries is not important. Those should be assigned
//      later. Of course assigning the correct values in this step is also
//      possible.
//      -> To assign a sparsity pattern, use the method addSubMatrix. The given
//      matrix should be one without zero values.
//      -> After all entries have been added, call assemble(). After that,
//      addSubMatrix() must not be called anymore.
// Usage:
//      -> before starting to add values, call setZero() to set every value zero.
//      This doesn't change the sparsity pattern and can be called safely.
//      -> to update a 3x3 sub matrix at (row, column), either retrieve the
//      matrices 3 column starting indices with getColumnIndices() and pass
//      them along with the to be added values to addColumn(),
//      or
//      a faster way: directly write the sub matrix columns memory locations
//      which can be obtained with createColumnsPtr(). Make sure to call this
//      method only once in the initalization as it allocates memory each time
//      its called.
//
// How does it work:
//
// Take this matrix as an example:
//   0 1 2 3 4 5 6 7 8
// 0 o o o o o o o o o
// 1 o o o o o o o o o
// 2 o o o o o o o o o
// 3 o o o x x x o o o
// 4 o o o x x x o o o
// 5 o o o x x x o o o
// 6 o o o y y y o o o
// 7 o o o y y y o o o
// 8 o o o y y y o o o
//
// "o" are zero entries while "x" and "y" are entries of two sub 3x3 sub
// matrices X and Y. Assemble this matrix once in the initalization. Note that
// the actual value of X and Y don't matter:
//
// SparseMatrix33 m(3,3);
// m.addSubMatrix(1,1); // sub matrix X
// m.addSubMatrix(2,1); // sub matrix Y
// // creates the full sparse matrix of type Eigen::SparseMatrix<double> as it can be seen above
// m.assemble();
//
// // access
// // first way
// const std::array<Eigen::Index, 3>& subX = m.getColumnIndices(1,1);
// const std::array<Eigen::Index, 3>& subY = m.getColumnIndices(2,1);
// m.addColumn(subX[0], Eigen::Vector3d(1.0, 2.0, 3.0));
// m.addColumn(subX[1], Eigen::Vector3d(4.0, 5.0, 6.0));
// m.addColumn(subX[2], Eigen::Vector3d(7.0, 8.0, 9.0));
// m.addColumn(subY[0], Eigen::Vector3d(9.0, 8.0, 7.0));
// m.addColumn(subY[1], Eigen::Vector3d(6.0, 5.0, 4.0));
// m.addColumn(subY[2], Eigen::Vector3d(3.0, 2.0, 1.0));
// m.addColumn(subY[2], Eigen::Vector3d(4.0, 3.0, 5.0)); // these values are adding up
//
// // second way
// std::array<double*, 3> subX = m.createColumnPtrs(1,1);
// std::array<double*, 3> subY = m.createColumnPtrs(2,1);
// Eigen::Map<Eigen::Vector3d>(subX[0]) = Eigen::Vector3d(1.0, 2.0, 3.0);
// Eigen::Map<Eigen::Vector3d>(subX[1]) = Eigen::Vector3d(4.0, 5.0, 6.0);
// Eigen::Map<Eigen::Vector3d>(subX[2]) = Eigen::Vector3d(7.0, 8.0, 9.0);
// Eigen::Map<Eigen::Vector3d>(subY[0]) = Eigen::Vector3d(9.0, 8.0, 7.0);
// Eigen::Map<Eigen::Vector3d>(subY[1]) = Eigen::Vector3d(6.0, 5.0, 4.0);
// Eigen::Map<Eigen::Vector3d>(subY[2]) = Eigen::Vector3d(3.0, 2.0, 1.0);
// Eigen::Map<Eigen::Vector3d>(subY[2]) = Eigen::Vector3d(4.0, 3.0, 5.0); // these values are adding up
//
// The result is the following matrix (which can be obtained by calling
// getMatrix()):
//
//   0 1 2 3 4 5 6 7 8
// 0 o o o o o o o o o
// 1 o o o o o o o o o
// 2 o o o o o o o o o
// 3 o o o 1 2 3 o o o
// 4 o o o 4 5 6 o o o
// 5 o o o 7 8 9 o o o
// 6 o o o 9 8 7 o o o
// 7 o o o 6 5 4 o o o
// 8 o o o 7 5 6 o o o
//
class SparseMatrix33
{
public:

    // The actual size of the matrix will be (3 * rows, 3 * columns).
    //\param rows - number of rows of 3d matrices
    //\param column - number of columns of 3d matrices
    SparseMatrix33(Eigen::Index rows, Eigen::Index columns);

    // Matrix Construction method
    void addSubMatrix(Eigen::Index row,
                      Eigen::Index column,
                      const Eigen::Matrix3d& matrix = Eigen::Matrix3d::Ones());

    // Matrix Construction method
    void assemble();

    // Returns 3 indices that point to the starting points
    // of the columns of the 3x3 matrix that is at (row, column).
    const std::array<Eigen::Index, 3>& getColumnIndices(
            Eigen::Index row,
            Eigen::Index column);

    // Returns pointers to the memory locations of getColumnIndices(). Using
    // these pointers, the memory can be updated directly. This is the fastest
    // way of updating but also the most unsecure. (A more secure way would
    // be by doing the update with indices via addColumn()).
    //
    // -> This method allocates new memory, so, it should be called only once
    // in the initialization. It only has to be recalled, if the sparsity
    // pattern of this matrix changed.
    // -> It has to made sure that for each pointer only at most 3 double
    // values are written, e.g.
    //
    // This is fine:
    //
    // std::array<double*, 3> columnPtrs = createColumnPtrs(0, 0);
    // for (int i = 0; i < 3; ++i)
    //      columnPtrs[0][0] = 0.0;
    //
    // or
    //
    // Eigen::Map<Eigen::Vector3d>(columnPtrs[0][0]).setZero();
    //
    // This would be illegal:
    //
    // std::array<double*, 3> columnPtrs = createColumnPtrs(0, 0);
    // for (int i = 0; i < 4; ++i) // overflow, there is no forth element!
    //      columnPtrs[0][0] = 0.0;
    //
    // or
    //
    // // overflow, there is no forth element!
    // Eigen::Map<Eigen::Vector4d>(columnPtrs[0][0]).setZero();
    //
    std::array<double*, 3> createColumnPtrs(
            Eigen::Index row,
            Eigen::Index column);

    // Add a column that is at the given internal column index.
    // This index must be one that was obtained by getColumnIndices().
    void addColumn(Eigen::Index internalColumnIndex,
                   const Eigen::Vector3d& column);

    void setZero();

    Eigen::SparseMatrix<double>& getMatrix();

private:
    typedef std::pair<Eigen::Index, Eigen::Index> RowColumnPair;

    // Column major sparse matrix. This means that it allows
    // an efficient access of sub columns.
    Eigen::SparseMatrix<double, Eigen::ColMajor> mMatrix;

    // [points to entry in large matrix containing 3x3 sub matrices]
    // (row, column) ->
    // 3 indices that point at the starting value of the 3 columns of
    // the 3x3 matrix behind (row, column)
    std::map<RowColumnPair, std::array<Eigen::Index, 3>> mIndexMap;

    // Use for the construction of the matrix.
    std::vector<Eigen::Triplet<double, Eigen::Index>> mTriplets;
};

#endif // SPARSEMATRIX33_H
