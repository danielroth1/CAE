#include "ConstraintManager.h"

#include "Constraint.h"
#include "Truncation.h"

ConstraintManager::ConstraintManager()
{

}

LinearForce* ConstraintManager::createLinerForceConstraint()
{
    return NULL;
}

Truncation* ConstraintManager::createTruncation()
{
    return NULL;
}

void ConstraintManager::removeConstraint(int id)
{
    m_constraint_id_map.erase(id);
}

Constraint *ConstraintManager::getConstraint(int id)
{
    return m_constraint_id_map[id];
}
