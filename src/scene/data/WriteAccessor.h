#ifndef WRITEACCESSOR_H
#define WRITEACCESSOR_H

#include "Accessor.h"


class WriteAccessor : public Accessor
{
public:
    WriteAccessor(boost::shared_mutex& mutex);
    virtual ~WriteAccessor();


    // Accessor interface
public:
    virtual void start();
    virtual void finish();
};

#endif // WRITEACCESSOR_H
