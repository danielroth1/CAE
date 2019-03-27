#ifndef LINEARFORCE_H
#define LINEARFORCE_H

#include "Constraint.h"
#include "data_structures/DataStructures.h"
#include "ui/UniqueVertex.h"

#include <simulation/fem/FEMObject.h>

#include <simulation/references/SimulationPointRef.h>

// For now, this class holds the target vertex as Vector
// and not as UniqueVertex
class LinearForce : public Constraint
{
public:
    LinearForce(
            SimulationPointRef source,
            SimulationPointRef target,
            double strength);

    virtual ~LinearForce() override;

    // Getter
    const SimulationPointRef& getTargetVector() const;
    const SimulationPointRef& getSourceVector() const;
    double getStrength() const;

    // Setter
    void setTargetVector(SimulationPointRef target);
    void setSourceVector(SimulationPointRef source);
    void setStrength(double strength);

    // Visitor method
    virtual void accept(ConstraintVisitor& cv) override;
    virtual bool references(Constraint* c) override;
    virtual bool references(SimulationObject* so) override;

private:

    SimulationPointRef mSource;
    SimulationPointRef mTarget;
    double mStrength;

//    SimulationObject* mSource;
//    ID mSourceVectorID;

//    Eigen::Vector mTargetVertex;
//    double mStrength; // (depends on distance?)
};

#endif // LINEARFORCE_H
