
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
    if (mTruncatedIds.empty())
        return;

    truncateByRemoving(A, ATrunc);
    truncateByRemoving(b, bTrunc);
}

void Truncation::truncateByRemoving(
        const Eigen::SparseMatrix<double>& A,
        Eigen::SparseMatrix<double>& ATrunc)
{
    if (mTruncatedIds.empty())
        return;

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
    if (mTruncatedIds.empty())
        return;

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

void Truncation::analyzePattern(Eigen::SparseMatrix<double>& A)
{
    if (mTruncatedIds.empty())
        return;

    A.makeCompressed();
    // Not the most efficient implementation but since it is only called once
    // in the initialization it shouldn't matter.
    std::set<Eigen::Index> truncatedIdsSet;
    for (ID id : mTruncatedIds)
    {
        truncatedIdsSet.insert(static_cast<Eigen::Index>(id * 3));
        truncatedIdsSet.insert(static_cast<Eigen::Index>(id * 3 + 1));
        truncatedIdsSet.insert(static_cast<Eigen::Index>(id * 3 + 2));
    }

    mSetZeroIndices.clear();
    mSetOneIndices.clear();

    int* outerIndices = A.outerIndexPtr();
    int* rowIndices = A.innerIndexPtr();

    for (int c = 0; c < A.cols(); ++c)
    {
//        int elementCount = elementCounts[c];
        int elementCount = outerIndices[c+1] - outerIndices[c];
        int startingElementsIndex = outerIndices[c];

        for (int i = 0; i < elementCount; ++i)
        {
            int memoryIndex = startingElementsIndex + i;
            int r = rowIndices[memoryIndex];

            if (truncatedIdsSet.find(r) != truncatedIdsSet.end() ||
                truncatedIdsSet.find(c) != truncatedIdsSet.end())
            {
                if (r == c)
                    mSetOneIndices.push_back(memoryIndex);
                else
                    mSetZeroIndices.push_back(memoryIndex);
            }
        }
    }
}

void Truncation::truncateBySettingZero(
        Eigen::SparseMatrix<double>& A,
        const VectorXd& b,
        Eigen::SparseMatrix<double>& ATrunc,
        VectorXd& bTrunc)
{
    if (mTruncatedIds.empty())
        return;

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
    ATrunc.setFromTriplets(coefficients.begin(), coefficients.end());
}

void Truncation::truncateBySettingZeroSlow(Eigen::SparseMatrix<double>& A, VectorXd& b)
{
    for (ID id : mTruncatedIds)
    {
        for (Eigen::Index r = 0; r < 3; ++r)
        {
            Eigen::Index r_global = static_cast<Eigen::Index>(id) * 3 + r;
            // truncation of b
            b(r_global) = 0.0;

            // truncation of A
            for (Eigen::Index c = 0; c < b.size(); ++c)
            {
                Eigen::Index c_global = c;
                // inefficient but sufficient for now
                if (r_global == c_global)
                    A.coeffRef(r_global, c_global) = 1.0;
                else
                {
                    A.coeffRef(r_global, c_global) = 0.0;
                    A.coeffRef(c_global, r_global) = 0.0;
                }
            }

        }
    }
}

void Truncation::truncateBySettingZeroFast(
        Eigen::SparseMatrix<double>& A,
        VectorXd& b)
{
    if (mTruncatedIds.empty())
        return;

    A.makeCompressed();
    for (const Eigen::Index& index : mSetOneIndices)
    {
        A.valuePtr()[index] = 1.0;
    }
    for (const Eigen::Index& index : mSetZeroIndices)
    {
        A.valuePtr()[index] = 0.0;
    }
    truncateBySettingZeroFast(b);
}

void Truncation::truncateBySettingZeroFast(VectorXd& b)
{
    if (mTruncatedIds.empty())
        return;

    for (ID id : mTruncatedIds)
    {
        for (Eigen::Index r = 0; r < 3; ++r)
        {
            Eigen::Index r_global = static_cast<Eigen::Index>(id) * 3 + r;
            // truncation of b
            b(r_global) = 0.0;
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
