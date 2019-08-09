#ifndef TIMING_H
#define TIMING_H


#include <chrono>
#include <iostream>
#include <map>
#include <stack>
#include <string>
#include <vector>


// Use the in this class provided macros to do some in code performance
// profiling analysis.
// The following macros allow to capture the number of calls and
// the passed time of each call. A call starts with
// START_TIMING(<call-name>, <#instance>) and ends with STOP_TIMING(<#instance>)
//
// When the program is closed or somewhere inbetween when felt necessary use
// PRINT_TIMING to print the up to this point captured timing statistics.
//
// Call operations can be grouped in instances. Timings are printed all at once
// per instance.
// There can be multiple call operations per instance. Call operations of one
// instance must be used in the same thread to avoid any race conditions.
// It is recomended to define an own macro (how its done below) for each instance
// to increase readibily, so it is possible to write in the simulation thread:
//
// START_TIMING_SIMULATION(<call-name>)
// <call-code>
// STOP_TIMING_SIMULATION
//
// instead of
//
// START_TIMING(<call-name>, 1)
// <call-code>
// STOP_TIMING(1)
namespace times
{
    #define START_TIMING(timerName, instance) \
        times::Timing::getInstance(instance)->startTiming(timerName);

    #define STOP_TIMING(instance) \
        times::Timing::getInstance(instance)->stopTiming();

    #define INIT_TIMING(instance) \
        times::Timing::getInstance(instance)->restart();

    #define PRINT_TIMING \
        times::Timing::printAll();

// module specific macros
    #define START_TIMING_RENDERING(timerName) \
        times::Timing::getInstance(0)->startTiming(timerName);

    #define STOP_TIMING_RENDERING \
        times::Timing::getInstance(0)->stopTiming();

    #define START_TIMING_SIMULATION(timerName) \
        times::Timing::getInstance(1)->startTiming(timerName);

    #define STOP_TIMING_SIMULATION \
        times::Timing::getInstance(1)->stopTiming();

    class Timing
    {
    public:

        typedef std::chrono::time_point<std::chrono::high_resolution_clock> TimePoint;
        typedef std::chrono::duration<double> Duration; // duration in seconds
        typedef std::time_t Time;

        static Timing* getInstance(size_t n)
        {
            if (n >= mInstances.size())
            {
                for (size_t i = mInstances.size(); i < n + 1; ++i)
                {
                    mInstances.push_back(new Timing());
                }
            }
            return mInstances[n];
        }

        static std::vector<Timing*>& getInstances()
        {
            return mInstances;
        }

        inline void restart()
        {
            mTimingMap.clear();
            while (!mStack.empty())
            {
                mStack.pop();
            }
            mTimingOrder.clear();
        }

        inline TimePoint getCurrentTime()
        {
            return std::chrono::system_clock::now();
        }

//        inline static Time toTime(TimePoint& timePoint)
//        {
//            return std::chrono::system_clock::to_time_t(timePoint);
//        }

        inline double toSeconds(Duration duration)
        {
            return duration.count();
        }

//        inline static std::chrono::milliseconds toTimeMS(Duration duration)
//        {
//            return std::chrono::duration_cast<std::chrono::milliseconds>(duration);
//        }

        inline void startTiming(const std::string& name = std::string(""))
        {
            mStack.push(TimingNode(name, getCurrentTime()));

            auto it = mTimingMap.find(name);
            if (it == mTimingMap.end())
            {
                mTimingMap[name] = std::make_tuple(0, 0.0, mStack.size());
                mTimingOrder.push_back(name);
            }
        }

        inline void stopTiming()
        {
            TimingNode& node = mStack.top();
            Duration executionTime = getCurrentTime() - node.mStartingTime;
            double executionTimeSeconds = toSeconds(executionTime);

            auto it = mTimingMap.find(node.mName);
            if (it != mTimingMap.end())
            {
                std::get<0>(it->second) += 1;
                std::get<1>(it->second) += executionTimeSeconds;
            }

            mStack.pop();
        }

        inline void print()
        {
            for (const std::string& name : mTimingOrder)
            {
                const std::tuple<size_t, double, size_t>& t = mTimingMap[name];
                size_t nCalls = std::get<0>(t);
                double totalTime = std::get<1>(t);
                double averageTime = totalTime / nCalls;
                size_t depth = std::get<2>(t);

                for (size_t i = 0; i < depth; ++i)
                {
                    std::cout << "...";
                }
                std::cout << name << ": " << averageTime * 1000 << "(ms) /  " << totalTime << " / " << nCalls << std::endl;
            }
        }

        inline static void printAll()
        {
            for (size_t i = 0; i < Timing::getInstances().size(); ++i)
            {
                std::cout << "instance #" << i << std::endl;
                Timing::getInstances()[i]->print();
            }
        }

    private:

        static std::vector<Timing*> mInstances;

        struct TimingNode
        {
            TimingNode(const std::string& _name, TimePoint _startingTime)
                : mName(_name)
                , mStartingTime(_startingTime)
            {
                std::chrono::system_clock::to_time_t(mStartingTime);
            }

            std::string mName;
            TimePoint mStartingTime;
        };

        // timing name, number of calls, total execution time(seconds), depth
        std::map<std::string, std::tuple<size_t, double, size_t>> mTimingMap;

        std::stack<TimingNode> mStack;

        std::vector<std::string> mTimingOrder;
    };
}

#endif // TIMING_H
