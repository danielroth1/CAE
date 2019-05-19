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
    auto queue = mQueue.lock();
    queue->push(op);
}

void Domain::processOperations()
{
    while (!mQueue.unsafe().empty())
    {
        // Only adding/ removing the queue must be threadsafe
        // It is important that there is not lock while operations are
        // called becaues adding an operation wihtin that call
        // would result in a deadlock
        Operation* op;
        {
            auto queue = mQueue.lock();
            // retrieve first element and call operation
            op = queue->front();
            queue->pop();
        }
        op->call();
        delete op;

    }
}

void Domain::setThreadId(std::thread::id threadId)
{
    mThreadId = threadId;
}

std::thread::id Domain::getThreadId() const
{
    return mThreadId;
}
