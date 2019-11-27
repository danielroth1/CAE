#ifndef TOPOLOGYFEATUREITERATOR_H
#define TOPOLOGYFEATUREITERATOR_H


#include <vector>

class TopologyFeature;

class TopologyFeatureIterator
{
public:
    TopologyFeatureIterator();
    virtual ~TopologyFeatureIterator();

    virtual TopologyFeatureIterator& operator++();

    virtual TopologyFeature& operator*() = 0;

    // Resets the iterator to the starting point.
    virtual void reset() = 0;

    virtual size_t getSize() =0;

};

#endif // TOPOLOGYFEATUREITERATOR_H
