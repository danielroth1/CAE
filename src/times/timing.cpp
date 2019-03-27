
#include "timing.h"

using namespace times;

std::map<std::string, std::tuple<size_t, double, size_t>> Timing::mTimingMap;
std::stack<Timing::TimingNode> Timing::mStack;
std::vector<std::string> Timing::mTimingOrder;
