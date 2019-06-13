#ifndef VECTORINDEXMAPPING_H
#define VECTORINDEXMAPPING_H

#include <string>
#include <vector>

// If we have two vectors that represent the same data but one vector
// needs to store duplications of some of the elements, then this class
// stores the mapping between the original vector and the extended one.
//
// original =   0, 1, 2, 3, 4, 5, 6
// extended =   0, 1, 1, 1, 2, 3, 3, 4, 5, 5, 5, 6
// count =      0, 2, 0, 1, 0, 2, 0
// startIndex = 0, 1, 4, 5, 7, 8, 11
//
// accum =      0, 2, 2, 3, 3, 5, 5
// accum =      0, 3, 3, 5, 5, 7, 7
// accum =      1, 4, 5, 7, 8, 11, 12
//
// // Returns the first index
// getExtendedIndex(original) {
//      vector<int> indices;
//      for (int i = 0; i < count(original); ++i)
//          indices.push_back(original + accum(original))
//
// getOriginalIndex(3) = 1
// getOriginalIndex(4) = 2
// getExtendedIndex(1) = 1 // {1, 2, 3}
// getExtendedIndex(4) = 7 // {7}
// getExtendedIndex(5) = 8
// count(i) = getExtended(i).size() - 1
//
// getOriginalIndex(size_t extended)
// getExtendedIndex(size_t original)
//
// Maps the elements of two vector against each other.
// - A.size() <= B.size()
// - A does not change in size
// - B can only grow
// For two vectors A and B , it is:
// getAToB() = vector of indices in B
// B[getBToA()]
//
// Itearte over all duplicated indices of i by:
//
// if (getNumberOfDuplicatedIndices(i)) {
//   std::size_t startingIndex = getStartingExtendedIndex(i);
//   for (size_t j = 0; j < getNumberOfDuplicatedIndices(i); ++j)
//       std::size_t index = startingIndex + j;
// }
class VectorIndexMapping
{
public:

    VectorIndexMapping(std::size_t initialSize);

    void reset(std::size_t initialSize);

    // Call this method if duplicateIndex was called once before calling one of
    // the gettesr.
    // Calculates and sets extended start indices.
    void revalidate();

    // Time complexity O(1)
    // With this method, the state remains consistent.
    // Call revalidate() after calling this method once before using any of
    // the getters. This approach assures a O(1) time complexity for this method.
    void duplicateIndex(std::size_t originalIndex);

    std::size_t getOriginalSize() const;
    std::size_t getExtendedSize() const;

    std::size_t getOriginalIndex(std::size_t extendedIndex) const;

    // Returns the first index in the extended vector at which the
    // duplicated originalIndices are stored. Use this method in conjunction with
    // getCount(originalIndex) to itearte over all duplicates.
    std::size_t getStartingExtendedIndex(std::size_t originalIndex) const;

    // Returns the number of times the given index ist stored additinally which
    // is the total number of times stored minus one.
    // If its stored 4 times, this method returns 3.
    std::size_t getNumDuplicatedIndices(std::size_t originalIndex) const;

    std::string toString();

private:
    std::size_t mOriginalSize;

    // extended size
    // stores corresponding original indices
    std::vector<std::size_t> mExtendedToOriginalIndices; // stores at [i] the corrensponding orignal index

    // original size
    // stores for each original index the starting point in the extended vector
    std::vector<std::size_t> mExtendedStartIndices;

    // original size
    // stores for each original index the number of extended indices
    std::vector<std::size_t> mNumDuplicatedIndices;
};

#endif // VECTORINDEXMAPPING_H
