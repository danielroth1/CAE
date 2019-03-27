#ifndef RIGIDBODYPOINTREF_H
#define RIGIDBODYPOINTREF_H

#include <Eigen/Core>

class RigidBody;

class RigidBodyPointRef
{
public:
    RigidBodyPointRef(RigidBody* rb);

    // Getterst
    RigidBody* getRigidBody();
    Eigen::VectorXd& getR();

private:
    RigidBody* mRigidBody;

    // Vector that points from center of mass to the vector
    Eigen::VectorXd mR;
};

#endif // RIGIDBODYPOINTREF_H
