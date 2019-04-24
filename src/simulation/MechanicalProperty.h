#ifndef MECHANICALPROPERTY_H
#define MECHANICALPROPERTY_H

#include <memory>

class SimulationObject;

class MechanicalProperty : public std::enable_shared_from_this<MechanicalProperty>
{
public:
    MechanicalProperty();
    virtual ~MechanicalProperty();

    virtual bool references(const std::shared_ptr<SimulationObject>& so) = 0;
};

#endif // MECHANICALPROPERTY_H
