#ifndef SIMULATIONDATAVISITOR_H
#define SIMULATIONDATAVISITOR_H

#include <scene/data/simulation/FEMData.h>


class SimulationDataVisitor
{
public:

    virtual void visit(FEMData& femData) = 0;

protected:
    SimulationDataVisitor() { }
    virtual ~SimulationDataVisitor();
};

SimulationDataVisitor::~SimulationDataVisitor()
{
}

#endif // SIMULATIONDATAVISITOR_H
