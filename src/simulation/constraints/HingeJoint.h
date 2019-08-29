#ifndef HINGEJOINT_H
#define HINGEJOINT_H

#include "CombinedConstraint.h"
#include <data_structures/DataStructures.h>

class RigidBody;

class HingeJoint : public CombinedConstraint
{
public:
    HingeJoint(
            const std::shared_ptr<RigidBody>& rbA,
            const std::shared_ptr<RigidBody>& rbB,
            Eigen::Vector pointABS,
            Eigen::Vector pointBBS,
            Eigen::Vector axis1BS,
            Eigen::Vector axis2BS);

    virtual void accept(ConstraintVisitor& cv);
};

#endif // HINGEJOINT_H
