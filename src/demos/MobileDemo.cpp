#include "MobileDemo.h"

#include <ApplicationControl.h>

#include <simulation/rigid/RigidBody.h>

#include <simulation/constraints/BallJoint.h>

#include <simulation/forces/LinearForce.h>

#include <scene/data/geometric/GeometricPoint.h>

using namespace Eigen;

MobileDemo::MobileDemo(ApplicationControl& ac)
    : mAc(ac)
{

}

std::string MobileDemo::getName()
{
    return "Mobile";
}

void MobileDemo::load()
{

}

void MobileDemo::unload()
{
}
