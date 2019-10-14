#ifndef TRUNCATION_H
#define TRUNCATION_H

// Includes
#include "Constraint.h"
#include "data_structures/DataStructures.h"
#include <Eigen/Sparse>
#include <set>

// Stores truncated vector ids.
// Truncated vertices are those that are completely static. This is achieved
// by taking them out of the equation, either by setting their entires zero
// (truncateBySettingZeroFast()) or by completely removing them from the
// equation (truncateByRemoving()).
class Truncation
{
public:
    Truncation();

    Truncation(const std::vector<ID>& truncatedVectorIds);

    void addTruncationIds(const std::vector<ID>& ids);

    void removeTruncationIds(const std::vector<ID>& ids);

    // Removes all truncated ids.
    void clear();

    const std::vector<ID>& getTruncatedVectorIds() const;

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

    // Call this method each time the matrix pattern (non-zero entries)
    // change before calling truncateBySettingZero().
    void analyzePattern(Eigen::SparseMatrix<double>& A);

    // Truncates A by removing entries of A that correspond to the to be
    // truncated vector ids. Recreates the reduces matrix and vector and
    // returns them in ATrunc and bTrunc.
    // This way of truncation can be rather slow because a new matrix is
    // created. It could make sense if so many vertices are truncated that
    // the resulting matrix is small. Though, most of the time
    // truncateBySetttingZeroFast() should be preferred.
    void truncateBySettingZero(
            Eigen::SparseMatrix<double>& A,
            const Eigen::VectorXd& b,
            Eigen::SparseMatrix<double>& ATrunc,
            Eigen::VectorXd& bTrunc);

    // Truncates A by making all entries of all rows and columns at truncated
    // vector ids zero. Uses binary search to find the entries and is therefore
    // a lot slower compared to truncatedBySettingZeroFast().
    void truncateBySettingZeroSlow(
            Eigen::SparseMatrix<double>& A,
            Eigen::VectorXd& b);

    // Sets the truncated rows and entries of the given matrix and vector zero.
    // Operates directly on the given data structures and does not require
    // a recreation which makes it substantially faster than the other
    // truncation methods.
    void truncateBySettingZeroFast(
            Eigen::SparseMatrix<double>& A,
            Eigen::VectorXd& b);

    // Same as the other method but only operates on the vector.
    void truncateBySettingZeroFast(
            Eigen::VectorXd& b);

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

    std::vector<Eigen::Index> mSetZeroIndices;
    std::vector<Eigen::Index> mSetOneIndices;
};

#endif // TRUNCATION_H
