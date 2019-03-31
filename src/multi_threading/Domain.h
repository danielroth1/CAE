#ifndef DOMAIN_H
#define DOMAIN_H

// Includes

#include "Monitor.h"

#include <queue>


// Forward Declarations

class Operation;

// Is a special observer for operation events.
// Stores operations in a FILO queue. Empties the queue and calls
// the operations with processOperations().
// This class is used in Thread.
// Call processOperations() at the start of each while cycle.
class Domain
{
public:
    Domain();
    virtual ~Domain();

    // Delegated operation queue methods
    void addOperation(Operation* op);

    // Process all operations in the operation queue at once.
    // Call this method at one point in your while loop.
    void processOperations();

private:
    Monitor<std::queue<Operation*>> mQueue;
};

#endif // DOMAIN_H
