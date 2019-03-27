#ifndef SIMULATIONOBJECTVISITOR_H
#define SIMULATIONOBJECTVISITOR_H

class FEMObject;
class SimulationPoint;
class RigidBody;

class SimulationObjectVisitor
{
public:

    virtual void visit(FEMObject& femObject) = 0;
    virtual void visit(SimulationPoint& sp) = 0;
    virtual void visit(RigidBody& rigidBody) = 0;

protected:
    SimulationObjectVisitor();
    virtual ~SimulationObjectVisitor();
};

#endif // SIMULATIONOBJECTVISITOR_H

