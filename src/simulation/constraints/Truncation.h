#ifndef TRUNCATION_H
#define TRUNCATION_H

// Includes
#include "Constraint.h"
#include "data_structures/DataStructures.h"
#include <Eigen/Sparse>
#include <set>

// Stores truncated vector ids.
// There is one Truncation per SimulationObject.
// Modifies the give sparse matrix A and vector b.
// They can be retrieved with getTruncatedA() and
// getTruncatedB().
class Truncation
{
public:
    Truncation();

    Truncation(const std::vector<ID>& truncatedVectorIds);

    // Use this getter to adapt the truncated vector ids.
    std::vector<ID>& getTruncatedVectorIds();

    // Takes the matrix A of the linear equation system A * x = b
    // and truncates it according to the truncated vector ids from
    // this truncation object. Essentially, a truncation removes
    // for all truncated vector ids i all rows and columns i of
    // matrix A and entries of vector b which decreases the number
    // of rows and columns by the number of truncated vector ids.
    void truncateByRemoving(
            const Eigen::SparseMatrix<double>& A,
            const Eigen::VectorXd& b,
            Eigen::SparseMatrix<double>& ATrunc,
            Eigen::VectorXd& bTrunc);

    // truncateByRemoving but only for the vector
    void truncateByRemoving(
            const Eigen::SparseMatrix<double>& A,
            Eigen::SparseMatrix<double>& ATrunc);

    // truncateByRemoving but only for the vector
    void truncateByRemoving(
            const Eigen::VectorXd& b,
            Eigen::VectorXd& bTrunc);

    // Truncates A by making all entries of all rows and columns at truncated
    // vector ids zero. Use createTruncatedMatrixA() for a more efficient
    // approach that reduces the system of equations.
    void truncateBySettingZero(
            Eigen::SparseMatrix<double>& A,
            const Eigen::VectorXd& b,
            Eigen::SparseMatrix<double>& ATrunc,
            Eigen::VectorXd& bTrunc);

    // Takes a truncated vector that was created with
    // createTruncatedVectorB() and reverts the truncation. It
    // fills the entries that were removed with zeros, which means
    // the returned vector is of its original size.
    Eigen::VectorXd createOriginal(
            const Eigen::VectorXd& truncatedB);

private:

    Eigen::SparseMatrix<double> m_A_trunc;
    Eigen::VectorXd m_b_trunc;

    std::vector<ID> mTruncatedIds;
};

#endif // TRUNCATION_H
