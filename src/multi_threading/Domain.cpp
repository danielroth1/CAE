#include "Domain.h"
#include "Operation.h"

#include <iostream>

Domain::Domain()
{

}

Domain::~Domain()
{

}

void Domain::addOperation(Operation* op)
{
    mQueue.push(op);
}

void Domain::processOperations()
{
//    if (!mQueue.empty())
//        std::cout << "processing " << mQueue.size() << " operations\n";
    while (!mQueue.empty())
    {
        // retrieve first element and call operation
        Operation* op = mQueue.front();
        mQueue.pop();
        op->call();
        delete op;
    }
}
