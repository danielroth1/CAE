#ifndef SPARSEMATRIX33_H
#define SPARSEMATRIX33_H

#include <map>
#include <Eigen/Sparse>

// Represents a stiffness matrix that is composed of 3x3 sub matrices.
// Offers efficient methods to access these sub matrices.
// Internally stores a regular sparse matrix consisting of doubles that
// can be used for regular calculations.
//
// Allows for efficient access
// Sparse matrices can be constructed once. Then the memory can be accessed
// directly as long as no more element is inserted.
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
//      -> to update a 3x3 sub matrix at (row, column), retrieve the matrices
//      3 column starting indices with getColumnIndices(). Then update the
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
    //
    const std::array<Eigen::Index, 3>& getColumnIndices(
            Eigen::Index row,
            Eigen::Index column);

    // Add a column that is behind the given internal column index.
    // This index must be one obtained from getIndex().
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
