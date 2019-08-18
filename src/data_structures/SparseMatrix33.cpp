#include "SparseMatrix33.h"


SparseMatrix33::SparseMatrix33(
        Eigen::Index rows,
        Eigen::Index columns)
    : mMatrix(3 * rows, 3 * columns)
{

}

void SparseMatrix33::addSubMatrix(
        Eigen::Index row,
        Eigen::Index column,
        const Eigen::Matrix3d& matrix)
{
    for (int r = 0; r < 3; ++r)
    {
        for (int c = 0; c < 3; ++c)
        {
            mTriplets.push_back(
                        Eigen::Triplet<double, Eigen::Index>(
                            row * 3 + r,
                            column * 3 + c,
                            matrix(r, c)));
        }
    }
}

void SparseMatrix33::assemble()
{
    mMatrix.setFromTriplets(mTriplets.begin(), mTriplets.end());
    // This is very important. It makes so that there is no empty memory
    // inbetween entries.
    mMatrix.makeCompressed();

    // fill mIndexMap
    // is nullptr in compressed mode!
    int* outerIndices = mMatrix.outerIndexPtr();

    int* rowIndices = mMatrix.innerIndexPtr();

    // Take this matrix as an example:
    //   0 1 2 3 4 5 6 7 8
    // 0 o o o o o o o o o
    // 1 o o o o o o o o o
    // 2 o o o o o o o o o
    // 3 o o o x x x o o o <- 3 % 3 = 0
    // 4 o o o x x x o o o <- 4 % 3 = 1
    // 5 o o o x x x o o o <- 5 % 3 = 2
    // 6 o o o y y y o o o <- 6 % 3 = 0
    // 7 o o o y y y o o o <- 7 % 3 = 1
    // 8 o o o y y y o o o <- 8 % 3 = 2
    //
    // The goal is to find the starting indices of the columns of the 3x3
    // matrices (indicated by the "x" and "y" values). For the "x" matrix,
    // they are at (3,3), (3,4), and (3,5). For the "y" matrix they are at
    // (6,3), (6,4), and (6,5).
    // -> The row values for each matrix are the same.
    // -> A column starting point of a submatrix is always at a row index
    // dividable by 3 so if row % 3 is true.
    // -> The sub matrix index is then the row and column divided by 3 rounded
    // down. -> (floor(row/3), floor(column/3))
    //
    for (int c = 0; c < mMatrix.cols(); ++c)
    {
//        int elementCount = elementCounts[c];
        int elementCount = outerIndices[c+1] - outerIndices[c];
        int startingElementsIndex = outerIndices[c];

        int subC = static_cast<int>(std::floor(c / 3.0));

        for (int i = 0; i < elementCount; ++i)
        {
            int memoryIndex = startingElementsIndex + i;
            int r = rowIndices[memoryIndex];

            if (r % 3 != 0)
                continue;

            int subR = static_cast<int>(std::floor(r / 3.0));

            auto it = mIndexMap.find(std::make_pair(subR, subC));
            if (it == mIndexMap.end())
            {
                std::array<Eigen::Index, 3> indices;
                indices[0] = memoryIndex;
                mIndexMap[std::make_pair(subR, subC)] = indices;
            }
            else
            {
                it->second[c % 3] = memoryIndex;
            }
        }
    }
}

const std::array<Eigen::Index, 3>& SparseMatrix33::getColumnIndices(
        Eigen::Index row, Eigen::Index column)
{
    return mIndexMap[std::make_pair(row, column)];
}

void SparseMatrix33::addColumn(
        Eigen::Index internalColumnIndex,
        const Eigen::Vector3d& column)
{
    double* valuePtr = mMatrix.valuePtr();
//    valuePtr[internalColumnIndex] += column(0);
//    valuePtr[internalColumnIndex+1] += column(1);
//    valuePtr[internalColumnIndex+2] += column(2);
    Eigen::Map<Eigen::Vector3d>(&valuePtr[internalColumnIndex]) += column;
}

void SparseMatrix33::setZero()
{
    // can't call mMatrix.setZero() because this changes the number of non-zero
    // elements.
    double* valuePtr = mMatrix.valuePtr();
    Eigen::Map<Eigen::VectorXd>(valuePtr, mMatrix.nonZeros()).setZero();

    // This is most likely not as efficient as the top version with Eigen::Map.
//    int* outerIndices = mMatrix.outerIndexPtr();

//    for (int c = 0; c < mMatrix.cols(); ++c)
//    {
//        int elementCount = outerIndices[c+1] - outerIndices[c];
//        int startingElementsIndex = outerIndices[c];

//        for (int i = 0; i < elementCount; ++i)
//        {
//            int memoryIndex = startingElementsIndex + i;

//            valuePtr[memoryIndex] = 0.0;
//        }
//    }
}

Eigen::SparseMatrix<double>& SparseMatrix33::getMatrix()
{
    return mMatrix;
}
