#ifndef READACCESSOR_H
#define READACCESSOR_H

#include "Accessor.h"



class ReadAccessor : public Accessor
{
public:
    ReadAccessor(boost::shared_mutex& mutex);
    virtual ~ReadAccessor();

    // Accessor interface
public:
    virtual void start();
    virtual void finish();
};

#endif // READACCESSOR_H
