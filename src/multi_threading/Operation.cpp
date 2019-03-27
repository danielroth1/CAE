#include "Operation.h"


Operation::Operation(std::function<void ()> function)
    : mFunction(function)
{

}

void Operation::call()
{
    mFunction();
}
