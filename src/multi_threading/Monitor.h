#ifndef MONITOR_H
#define MONITOR_H

#include <mutex>
#include <utility>


template<class T>
class Monitor
{
public:
    template<typename ...Args>
    Monitor(Args&&... args)
        : mData(std::forward<Args>(args)...)
    {

    }

    struct MonitorHelper
    {
        MonitorHelper(Monitor* monitor)
            : mMonitor(monitor), mUniqueLock(monitor->mLock)
        {

        }

        T* operator->()
        {
            return &mMonitor->mData;
        }

        T& operator*()
        {
            return mMonitor->mData;
        }

        Monitor* mMonitor;
        std::unique_lock<std::mutex> mUniqueLock;
    };

    MonitorHelper operator->()
    {
        return MonitorHelper(this);
    }

    // Lock the data manually
    // On calling creates a mutex in a helper structure.
    // That mutex is removed when the struct is removed,
    // i.e. when the scope is left.
    //
    // Use this method in the following way:
    // auto data = monitor.lock() // before accessing the data.
    // Now, use data to call all operations you could call directly
    // on the data of this class that is of type T.
    MonitorHelper lock()
    {
        return MonitorHelper(this);
    }

    // Unsafe accessing the monitored data directly.
    // No mutex will be locked or freed. The overhead
    // of calling this method is minimal.
    T& unsafe()
    {
        return mData;
    }

private:
    std::mutex mLock;
    T mData;
};

#endif // MONITOR_H
