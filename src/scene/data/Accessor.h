#ifndef ACCESSOR_H
#define ACCESSOR_H

#include <mutex>
#include <boost/thread/shared_mutex.hpp>

class Accessor
{
public:
    Accessor(boost::shared_mutex& mutex);
    virtual ~Accessor();

    virtual void start() = 0;

    virtual void finish() = 0;

protected:

    boost::shared_mutex& mMutex;
};

#endif // ACCESSOR_H
