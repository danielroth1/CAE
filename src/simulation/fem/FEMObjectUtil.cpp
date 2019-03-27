#include "FEMObject.h"
#include "FEMObjectUtil.h"

using namespace Eigen;

FEMObjectUtil* FEMObjectUtil::instance()
{
    if (m_instance == nullptr)
        m_instance = new FEMObjectUtil();
    return m_instance;
}

void FEMObjectUtil::actForceOnHighestVertex(FEMObject *so, Vector force)
{
    so->getExternalForces()[getTopVertex(so)] += force;
}

void FEMObjectUtil::actGravity(FEMObject* so, double gravity)
{
    for (auto& f_ext : so->getExternalForces())
        f_ext += Vector(0, -gravity, 0);
}

size_t FEMObjectUtil::getTopVertex(FEMObject* so)
{
    double yPos = so->getPositions()[0](1);
    size_t highest = 0;
    for (size_t i = 0; i < so->getPositions().size(); ++i)
    {
        double yPosCur = so->getPositions()[i](1);
        if (yPosCur > yPos)
        {
            yPosCur = yPos;
            highest = i;
        }
    }
    return highest;
}

size_t FEMObjectUtil::getBottomVertex(FEMObject* so)
{
    double yPos = so->getPositions()[0](1);
    size_t highest = 0;
    for (size_t i = 0; i < so->getPositions().size(); ++i)
    {
        double yPosCur = so->getPositions()[i](1);
        if (yPosCur < yPos)
        {
            yPosCur = yPos;
            highest = i;
        }
    }
    return highest;
}

FEMObjectUtil::FEMObjectUtil()
{

}

FEMObjectUtil* FEMObjectUtil::m_instance = nullptr;
