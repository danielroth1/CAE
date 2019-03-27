#ifndef OPERATION_H
#define OPERATION_H

#include <functional>

// Macros
// TODO: define macro that allows to
// automatically create operations by taking
// functions and input parameters, using std::bind

// Wrapper for a bind std::function
// Do not call this Constructor. Instead use the provided macros.
// Call the function with call().

// TODO: can multiple functions with arbitrary template parameters
// be stored and called all at once?
class Operation
{
public:
    Operation(std::function<void()> function);

    void call();

private:
    std::function<void()> mFunction;
};

#endif // OPERATION_H
