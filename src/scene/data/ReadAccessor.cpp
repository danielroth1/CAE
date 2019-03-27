#include "ReadAccessor.h"

ReadAccessor::ReadAccessor(boost::shared_mutex& mutex)
    : Accessor(mutex)
{

}

ReadAccessor::~ReadAccessor()
{

}

void ReadAccessor::start()
{
    mMutex.lock_shared();
}

void ReadAccessor::finish()
{
    mMutex.lock_shared();
}
