#ifndef BUFFEREDDATA_H
#define BUFFEREDDATA_H


#include <Eigen/Core>
#include <multi_threading/Monitor.h>
#include <vector>

// Manages a multithreaded buffered vector that stores
// data of type T.
//
// T is the data that is stored in the vector.
// NT == NumericType is the numeric type of the data, e.g.
//   NumericType == float for Vectorf or
//   NumericType == unsigned int for array<unsgined int, 4>
// NE == nElements are the number of elements of T, e.g.
//   if T == Vectorfs, then nElements == 3 or
//   if T == array<unsigned int, 4>, then nElements == 4.
template <class T, class NT, unsigned int NE>
class BufferedData
{
public:

    // \param target - OpenGL target, e.g. GL_ARRAY_BUFFER, GL_ARRAY_BUFFER
    // \param usage - OpenGL usage, e.g. GL_STATIC_DRAW, GL_DYNAMIC_DRAW
    BufferedData(unsigned int target,
                 unsigned int usage);

    BufferedData(BufferedData<T, NT, NE>& bd);

    void initialize();

    void cleanup();

    void createBuffer();

    void refreshBuffer();

    void bindBuffer();

    Monitor<std::vector<T>>& getData();

    bool isInitialized() const;

    void setDataChanged(bool dataChanged);
    bool isDataChanged() const;

private:
    Monitor<std::vector<T>> mData;
    unsigned int mVbo;

    unsigned int mTarget; // e.g. GL_ARRAY_BUFFER, GL_ELEMENT_ARRAY_BUFFER
    unsigned int mUsage; // e.g. GL_STATIC_DRAW, GL_DYNAMIC_DRAW

    bool mInitialized;
    bool mDataChanged;
};

#include "BufferedData.cpp"

#endif // BUFFEREDDATA_H
