#ifndef CONSTRAINTMANAGER_H
#define CONSTRAINTMANAGER_H

#include <map>

class Constraint;
class LinearForce;
class Truncation;

class ConstraintManager
{
public:
    ConstraintManager();

    LinearForce* createLinerForceConstraint();

    Truncation* createTruncation();

    void removeConstraint(int id);

    Constraint* getConstraint(int id);

private:
    std::map<int, Constraint*> m_constraint_id_map;

};

#endif // CONSTRAINTMANAGER_H
