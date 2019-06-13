#include "VectorIndexMapping.h"

#include <sstream>

VectorIndexMapping::VectorIndexMapping(std::size_t initialSize)
    : mOriginalSize(initialSize)
{
    reset(initialSize);
}

void VectorIndexMapping::reset(std::size_t initialSize)
{
    mExtendedToOriginalIndices.reserve(initialSize);
    mExtendedStartIndices.reserve(initialSize);
    mNumDuplicatedIndices.reserve(initialSize);

    for (std::size_t i = 0; i < initialSize; ++i)
    {
        mExtendedToOriginalIndices.push_back(i);
        mExtendedStartIndices.push_back(i);
        mNumDuplicatedIndices.push_back(0);
    }
}

void VectorIndexMapping::revalidate()
{
    mExtendedStartIndices.clear();
    mExtendedStartIndices.reserve(mOriginalSize);
    for (std::size_t i = 0; i < mOriginalSize; ++i)
    {
        std::size_t index = 0;
        if (i > 0)
            index = mExtendedStartIndices[i-1] + mNumDuplicatedIndices[i-1] + 1;
        mExtendedStartIndices.push_back(i);
    }
}

void VectorIndexMapping::duplicateIndex(std::size_t originalIndex)
{
    mExtendedToOriginalIndices.insert(
                mExtendedToOriginalIndices.begin() +
                static_cast<long>(getStartingExtendedIndex(originalIndex)),
                originalIndex);

    for (std::size_t i = originalIndex + 1; i < mExtendedStartIndices.size(); ++i)
    {
        ++mExtendedStartIndices[i];
    }

    ++mNumDuplicatedIndices[originalIndex];
}

std::size_t VectorIndexMapping::getOriginalSize() const
{
    return mOriginalSize;
}

std::size_t VectorIndexMapping::getExtendedSize() const
{
    return mExtendedToOriginalIndices.size();
}

std::size_t VectorIndexMapping::getOriginalIndex(std::size_t extendedIndex) const
{
    return mExtendedToOriginalIndices[extendedIndex];
}

std::size_t VectorIndexMapping::getStartingExtendedIndex(std::size_t originalIndex) const
{
    return mExtendedStartIndices[originalIndex];
}

std::size_t VectorIndexMapping::getNumDuplicatedIndices(std::size_t originalIndex) const
{
    return mNumDuplicatedIndices[originalIndex];
}

std::string VectorIndexMapping::toString()
{
    std::stringstream ss;

    ss << "Extended to Original indices:\t";
    for (std::size_t i = 0; i < mExtendedToOriginalIndices.size(); ++i)
    {
        ss << mExtendedToOriginalIndices[i];
        if (i < mExtendedToOriginalIndices.size() - 1)
            ss << ", ";
    }
    ss << "\n";

    ss << "Extended start indices:\t";
    for (std::size_t i = 0; i < mExtendedStartIndices.size(); ++i)
    {
        ss << mExtendedStartIndices[i];
        if (i < mExtendedStartIndices.size() - 1)
            ss << ", ";
    }
    ss << "\n";

    ss << "Num duplicated indices:\t";
    for (std::size_t i = 0; i < mNumDuplicatedIndices.size(); ++i)
    {
        ss << mNumDuplicatedIndices[i];
        if (i < mNumDuplicatedIndices.size() - 1)
            ss << ", ";
    }
    ss << "\n";
    return ss.str();
}
