#include "Accessor.h"

Accessor::Accessor(boost::shared_mutex& mutex)
    : mMutex(mutex)
{
}

Accessor::~Accessor()
{

}
