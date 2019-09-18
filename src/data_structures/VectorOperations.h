#ifndef VECTOROPERATIONS_H
#define VECTOROPERATIONS_H

#include "DataStructures.h"

#include <array>
#include <iostream>
#include <memory>
#include <set>
#include <vector>

class VectorOperations
{
public:
    VectorOperations();

    // Returns a vector with elements a \ b or a - b.
    template<class T>
    static std::vector<T> sub(const std::vector<T>& a,
                              const std::set<T>& b)
    {
        // Add all elements of a that are not in b.
        std::vector<ID> retVector;
        for (ID id : a)
        {
            if (b.find(id) == b.end())
            {
                retVector.push_back(id);
            }
        }
        return retVector;
    }

    // Removes the element at the given index from the given vector.
    template<class T>
    static void removeVector(std::vector<T>& vector, ID index)
    {
        vector.erase(vector.begin() + static_cast<int>(index));
    }

    // Removes all entries of the given vector at the given indices.
    //\param vector - the vector from which elements are removed.
    //\param begin - iterator pointing to the start of the indices.
    //\param end - iterator pointing to the end of the indices.
    template<class T, class BidirIter>
    static void removeVectors(
            std::vector<T>& vector, BidirIter begin, BidirIter end)
    {
        size_t removedCounter = 0;
        for (BidirIter it = begin; it != end; ++it)
        {
            size_t index = *it - removedCounter;
            if (index < vector.size())
                removeVector(vector, index);
            else
                std::cout << "Could not remove vector at " << index << " because"
                          << " its not that big. Possible explanation: passed "
                          << "too large indices or multiple indices twice to "
                          << "VectorOperations::removeVectors\n";
            ++removedCounter;
        }
    }

    // Removes all elements of arrays that contain at least one of the given
    // elements between begin and end, e.g.
    // arrays = [[0 1 2], [3 4 5], [6 7 8], [0 5 7]]
    // begin, end = [0 5]
    // afterwards:
    // arrays = [[6 7 8]]
    // -> removes   [0 1 2] because it contains 0,
    //              [3 4 5] because it contains 5,
    //              [0 5 7] because it contains 0 or 5
    template<class T, int N, class BidirIter>
    static void removeArrayIfContains(BidirIter begin,
                                      BidirIter end,
                                      std::vector<std::array<T, N>>& arrays)
    {
        for (size_t id = 0; id < arrays.size(); ++id)
        {
            bool remove = false;
            for (size_t i = 0; i < arrays[id].size(); ++i)
            {
                auto it = std::find(begin,
                                    end,
                                    arrays[id][i]);
                if (it != end)
                {
                    remove = true;
                    break;
                }
            }
            if (remove)
            {
                VectorOperations::removeVector(arrays, id);
                --id;
            }
        }
    }
};

#endif // VECTOROPERATIONS_H
