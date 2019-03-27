#include "WriteAccessor.h"

WriteAccessor::WriteAccessor(boost::shared_mutex& mutex)
    : Accessor(mutex)
{

}

WriteAccessor::~WriteAccessor()
{

}

void WriteAccessor::start()
{
    mMutex.lock();
}

void WriteAccessor::finish()
{
    mMutex.lock();
}
