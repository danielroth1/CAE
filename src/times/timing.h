#ifndef TIMING_H
#define TIMING_H


#include <chrono>
#include <iostream>
#include <map>
#include <stack>
#include <string>
#include <vector>

namespace times
{
    #define START_TIMING(timerName) \
        times::Timing::startTiming(timerName);

    #define STOP_TIMING \
        times::Timing::stopTiming();

    #define INIT_TIMING \
        times::Timing::restart();

    #define PRINT_TIMING \
        times::Timing::print();

    class Timing
    {
    public:

        typedef std::chrono::time_point<std::chrono::high_resolution_clock> TimePoint;
        typedef std::chrono::duration<double> Duration; // duration in seconds
        typedef std::time_t Time;

        inline static void restart()
        {
            mTimingMap.clear();
            while (!mStack.empty())
            {
                mStack.pop();
            }
            mTimingOrder.clear();
        }

        inline static TimePoint getCurrentTime()
        {
            return std::chrono::system_clock::now();
        }

//        inline static Time toTime(TimePoint& timePoint)
//        {
//            return std::chrono::system_clock::to_time_t(timePoint);
//        }

        inline static double toSeconds(Duration duration)
        {
            return duration.count();
        }

//        inline static std::chrono::milliseconds toTimeMS(Duration duration)
//        {
//            return std::chrono::duration_cast<std::chrono::milliseconds>(duration);
//        }

        inline static void startTiming(const std::string& name = std::string(""))
        {
            mStack.push(TimingNode(name, getCurrentTime()));

            auto it = mTimingMap.find(name);
            if (it == mTimingMap.end())
            {
                mTimingMap[name] = std::make_tuple(0, 0.0, mStack.size());
                mTimingOrder.push_back(name);
            }
        }

        inline static void stopTiming()
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

        inline static void print()
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
                std::cout << name << ": " << averageTime << " /  " << totalTime << " / " << nCalls << std::endl;
            }
        }

    private:
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
        static std::map<std::string, std::tuple<size_t, double, size_t>> mTimingMap;

        static std::stack<TimingNode> mStack;

        static std::vector<std::string> mTimingOrder;
    };
}

#endif // TIMING_H
