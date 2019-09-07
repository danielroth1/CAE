
// Includes
#include "ConstraintVisitor.h"
#include "Truncation.h"
#include <iostream>

// Forward Declarations
using namespace Eigen;

Truncation::Truncation()
{
}

Truncation::Truncation(const std::vector<ID>& truncatedVectorIds)
    : mTruncatedIds(truncatedVectorIds)
{
}


void Truncation::addTruncationIds(const std::vector<ID>& ids)
{
    for (ID vId : ids)
    {
        auto it = std::find(mTruncatedIds.begin(),
                            mTruncatedIds.end(),
                            vId);
        if (it == mTruncatedIds.end())
        {
            mTruncatedIds.push_back(vId);
        }
    }
}

void Truncation::removeTruncationIds(const std::vector<ID>& ids)
{
    for (ID vId : ids)
    {
        auto it = std::find(mTruncatedIds.begin(),
                            mTruncatedIds.end(),
                            vId);
        if (it != mTruncatedIds.end())
        {
            mTruncatedIds.erase(it);
        }
    }
}

void Truncation::clear()
{
    mTruncatedIds.clear();
}

const std::vector<ID>& Truncation::getTruncatedVectorIds() const
{
    return mTruncatedIds;
}

void Truncation::truncateByRemoving(
        const Eigen::SparseMatrix<double>& A,
        const Eigen::VectorXd& b,
        Eigen::SparseMatrix<double>& ATrunc,
        Eigen::VectorXd& bTrunc)
{
    truncateByRemoving(A, ATrunc);
    truncateByRemoving(b, bTrunc);
}

void Truncation::truncateByRemoving(
        const Eigen::SparseMatrix<double>& A,
        Eigen::SparseMatrix<double>& ATrunc)
{
    ID size = static_cast<ID>(A.rows());
    ID sizeTrunc = size - 3 * static_cast<ID>(mTruncatedIds.size());

    ATrunc = Eigen::SparseMatrix<double>(
                static_cast<long>(sizeTrunc),
                static_cast<long>(sizeTrunc));

    std::vector<Triplet<double>> coefficients;
    coefficients.reserve(static_cast<ID>(sizeTrunc) *
                         static_cast<ID>(sizeTrunc));

    ID kTrunc = 0;

    for (unsigned int k=0; k<A.outerSize(); ++k)
    {
        if (kTrunc < mTruncatedIds.size() && k >= mTruncatedIds[kTrunc] * 3)
        {
            kTrunc += 1;
            k += 2;
            continue;
        }

        ID rTrunc = 0;
        for (SparseMatrix<double>::InnerIterator it(A,k); it; ++it)
        {
            ID r = static_cast<ID>(it.row());
            //ID c = static_cast<ID>(it.col());
            double value = it.value();

            while (rTrunc < mTruncatedIds.size() &&
                   r > 3 * mTruncatedIds[rTrunc])
                rTrunc += 1;

            if (rTrunc < mTruncatedIds.size())
            {
                if (r == mTruncatedIds[rTrunc] * 3)
                {
                    rTrunc += 1;
                    ++it;
                    ++it;
                    continue;
                }
            }

            coefficients.push_back(
                        Triplet<double>(static_cast<int>(r - rTrunc * 3),
                                        static_cast<int>(k - kTrunc * 3),
                                        value));
        }
    }

    ATrunc.setZero();
    ATrunc.setFromTriplets(coefficients.begin(), coefficients.end());
}

void Truncation::truncateByRemoving(
        const VectorXd& b,
        VectorXd& bTrunc)
{
    ID size = static_cast<ID>(b.rows());
    ID sizeTrunc = size - 3 * static_cast<ID>(mTruncatedIds.size());

    bTrunc = VectorXd::Zero(
                static_cast<long>(sizeTrunc));

    std::vector<Triplet<double>> coefficients;
    coefficients.reserve(static_cast<ID>(sizeTrunc) *
                         static_cast<ID>(sizeTrunc));

    ID kTrunc = 0;

    for (unsigned int k=0; k < size; ++k)
    {
        if (kTrunc < mTruncatedIds.size() && k >= mTruncatedIds[kTrunc] * 3)
        {
            kTrunc += 1;
            k += 2;
            continue;
        }
        bTrunc(k - 3 * static_cast<long>(kTrunc)) = b(k);
    }
}

void Truncation::truncateBySettingZero(
        Eigen::SparseMatrix<double>& A,
        const VectorXd& b,
        Eigen::SparseMatrix<double>& ATrunc,
        VectorXd& bTrunc)
{
    ID size = static_cast<ID>(A.rows());
    ID sizeTrunc = size - 3 * static_cast<ID>(mTruncatedIds.size());

    ATrunc = SparseMatrix<double>(
                static_cast<long>(sizeTrunc),
                static_cast<long>(sizeTrunc));
    bTrunc = VectorXd::Zero(
                static_cast<long>(sizeTrunc));

    std::vector<Triplet<double>> coefficients;
    coefficients.reserve(sizeTrunc * sizeTrunc);

    unsigned int rTrunc = 0;
    for (unsigned int r = 0; r < size; ++r)
    {
        if (r == mTruncatedIds[rTrunc] * 3)
        {
            rTrunc += 2;
            continue;
        }

        bTrunc(r - rTrunc) = b(r);

        unsigned int cTrunc = 0;
        for (unsigned int c = 0; c < size; ++c)
        {
            if (c == mTruncatedIds[cTrunc])
            {
                cTrunc += 2;
                continue;
            }

            coefficients.push_back(
                        Triplet<double>(static_cast<int>(r - 3 * rTrunc),
                                        static_cast<int>(c - 3 * cTrunc),
                                        A.coeffRef(r,c)));
        }
    }
}

Eigen::VectorXd Truncation::createOriginal(
        const Eigen::VectorXd& truncatedB)
{
    ID size = static_cast<ID>(truncatedB.size()) + 3 * mTruncatedIds.size();
    VectorXd x = VectorXd::Zero(static_cast<long>(size));
    unsigned int iTrunc = 0;
    for (unsigned int i = 0; i < size; ++i)
    {
        if (iTrunc < mTruncatedIds.size() && i >= 3 * mTruncatedIds[iTrunc])
        {
            iTrunc += 1;
            i += 2;
            continue;
        }
        x(i) = truncatedB(i - iTrunc * 3);
    }
    return x;
}
