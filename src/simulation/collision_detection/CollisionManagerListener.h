#ifndef COLLISIONMANAGERLISTENER_H
#define COLLISIONMANAGERLISTENER_H


#include <memory>

class SimulationObject;

class CollisionManagerListener
{
public:

    virtual void notifySimulationObjectAdded(const std::shared_ptr<SimulationObject>& so) = 0;
    virtual void notifySimulationObjectRemoved(const std::shared_ptr<SimulationObject>& so) = 0;
    virtual void notifyCollideAllCalled() = 0;
    virtual void notifyUpdateAllCalled() = 0;

protected:
    CollisionManagerListener();
    virtual ~CollisionManagerListener();
};

#endif // COLLISIONMANAGERLISTENER_H
