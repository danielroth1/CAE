#ifndef TOPOLOGYFEATURE_H
#define TOPOLOGYFEATURE_H


class TopologyFeature
{
public:
    TopologyFeature();
    virtual ~TopologyFeature();

    enum class Type
    {
        VERTEX, EDGE, FACE
    };

    virtual Type getType() const = 0;
};

#endif // TOPOLOGYFEATURE_H
