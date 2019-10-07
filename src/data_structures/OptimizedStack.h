#ifndef OPTIMIZEDSTACK_H
#define OPTIMIZEDSTACK_H

#include <vector>

// A stack similar to std::stack but optimized so that no memory allocations
// or the usage of constructors are required.
// - Uses a std::vector internally to represent the stack. Memory is preallocated
//      using Ts default constructor (must be provided).
// - Preallocates the memory. The methods push() and pop() only edit a counter.
// - push() returns a reference to the top element that represents the added
//      element.
//
template <class T>
class OptimizedStack
{
public:
    OptimizedStack(size_t reservedSize = 1000)
    {
        // Add a dummy elemnt so its possible to access top element by
        // mStack[numElements] instead of mStack[numElements - 1] saving
        // a substraction operation.
        mStack.push_back(T());
        numElements = 0;

        reserve(reservedSize);
    }

    // Returns reference to top element.
    T& top()
    {
        return mStack[numElements];
    }

    // Adds given element. This will be the new top element.
    // \return reference to the element that is to be added.
    T& push()
    {
        ++numElements;
        return mStack[numElements];
    }

    // Removes the top element.
    void pop()
    {
        --numElements;
    }

    // Clears stack.
    void clear()
    {
        mStack.clear();
        numElements = 0;
    }

    // Returns number of elements in the stack.
    size_t size()
    {
        return numElements;
    }

    bool empty()
    {
        return numElements == 0;
    }

    // Reserves the given amount of memory.
    void reserve(size_t size)
    {
        mStack.resize(size);
    }

private:
    std::vector<T> mStack;
    size_t numElements;
};

#endif // OPTIMIZEDSTACK_H
