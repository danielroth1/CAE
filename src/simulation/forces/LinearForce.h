#ifndef LINEARFORCE_H
#define LINEARFORCE_H

#include "Force.h"
#include "data_structures/DataStructures.h"
#include "ui/UniqueVertex.h"

#include <simulation/fem/FEMObject.h>

#include <simulation/references/SimulationPointRef.h>

// For now, this class holds the target vertex as Vector
// and not as UniqueVertex
class LinearForce : public Force
{
public:
    LinearForce(
            SimulationPointRef source,
            SimulationPointRef target,
            double strength,
            double damping = 0.0,
            double length = 0.0);

    virtual ~LinearForce() override;

    virtual void applyForce() override;

    // Getter
    SimulationPointRef& getTargetVector();
    SimulationPointRef& getSourceVector();
    double getStrength() const;

    // Setter
    void setTargetVector(SimulationPointRef target);
    void setSourceVector(SimulationPointRef source);
    void setStrength(double strength);

    // Visitor method
    virtual bool references(const std::shared_ptr<SimulationObject>& so) override;

private:

    SimulationPointRef mSource;
    SimulationPointRef mTarget;
    double mStrength;
    double mDamping;
    double mLength;

//    SimulationObject* mSource;
//    ID mSourceVectorID;

//    Eigen::Vector mTargetVertex;
//    double mStrength; // (depends on distance?)
};

#endif // LINEARFORCE_H
