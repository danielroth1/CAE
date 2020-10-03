#include "RigidBody.h"

#include <simulation/SimulationObjectVisitor.h>

#include <math/MathUtils.h>

#include <scene/data/GeometricData.h>

#include <scene/data/geometric/AbstractPolygon.h>

#include <iostream>

#include <scene/data/references/GeometricPointRef.h>
#include <scene/data/references/GeometricPointRefVisitor.h>
#include <scene/data/references/GeometricVertexRef.h>
#include <scene/data/references/PolygonVectorRef.h>

#include <simulation/references/SimulationPointRef.h>

using namespace Eigen;

RigidBody::RigidBody(
        Domain* domain,
        std::shared_ptr<AbstractPolygon> polygon,
        Vectors& positions,
        double mass)
    : SimulationObject(domain, SimulationObject::Type::RIGID_BODY)
    , mPolygon(polygon)
    , mPositions(positions)
    , mMass(mass)
    , mStatic(false)
{
    mMassInv = 1.0 / mMass;

    std::vector<double> masses;
    masses.reserve(positions.size());
    for (size_t i = 0; i < positions.size(); ++i)
    {
        masses.push_back(mass);
    }
    mPolygon->changeRepresentationToBS(
                mPolygon->calculateCenterOfMass(masses));

    // calculate center of mass
    // since each vertex has the same mass, the
    // center of mass is equal to the average of vertices
//    mX = mPolygon->getCenter();

    mX = mPolygon->getTransform().translation();
    mQ = Eigen::Quaterniond(mPolygon->getTransform().rotation());

    // calculate inertia tensor from positions and faces
    mInertiaBS = Matrix3d::Zero();
    for (Vector& v : positions)
    {
        Vector r = v - mX;
        mInertiaBS += mass / positions.size() * (static_cast<double>(r.transpose() * r) * Matrix3d::Identity() -
                r * r.transpose());
    }
//    mInertiaBS.setIdentity(); // whats
    mInertiaInvBS = mInertiaBS.inverse();

    mOmega.setZero();
    mV.setZero();
    mTorqueExt.setZero();
    mForceExt.setZero();

    mTranslationalDamping = 0.005;
    mRotationalDamping = 0.005;

    update();
}

RigidBody::~RigidBody()
{

}

void RigidBody::update()
{
    if (mStatic)
        return;

    Eigen::Matrix3d rot = mQ.toRotationMatrix();
    mInertia = rot * mInertiaBS * rot.transpose();
    mInertiaInv = rot * mInertiaInvBS * rot.transpose();
}

void RigidBody::prepareNewStep()
{
    mForceExt.setZero();
    mTorqueExt.setZero();
}

void RigidBody::solveExplicit(double timeStep)
{
    if (mStatic)
        return;

    // 1.) integrate velocities:
    solveVelocityExplicit(timeStep);

    // 2.) integrate positions/rotations
    integratePositions(timeStep);
}

void RigidBody::solveVelocityExplicit(double timeStep)
{
    if (mStatic)
        return;

    // 1.) integrate velocities:
    // v_{i+1} = v_i + h * M^{-1} f_i
    mV = mV + timeStep * mMassInv * mForceExt;

    // \omega_{i+1} = \omega_i + h * I^{-1} (\tau_{ext} - (\omega_i \times (I \omega_i)))
    mOmega = mOmega + timeStep * mInertiaInv * (mTorqueExt - (mOmega.cross(mInertia * mOmega)));
}

void RigidBody::integratePositions(double timeStep)
{
    if (mStatic)
        return;

    mXOld = mX;
    mQOld = mQ;

    // x_{i+1} = x_i + h * v_{i+1}
    mX = mX + timeStep * mV;

    // q_{i+1} = q_i + h * 0.5 * toQuat(\omega_{i+1}) \otimes q_i
    mQ.coeffs() = mQ.coeffs() + timeStep *
            (Eigen::Quaterniond(0.0, 0.5 * mOmega(0), 0.5 * mOmega(1), 0.5 * mOmega(2))
             * mQ).coeffs();
    mQ.normalize();

    update();
}

void RigidBody::revertPositions()
{
    if (mStatic)
        return;

    mX = mXOld;
    mQ = mQOld;

    update();
}

void RigidBody::transform(const Affine3d& transform)
{
    mX = transform * mX;
    mQ = transform.rotation() * mQ;

    update();
}

void RigidBody::applyImpulse(SimulationPointRef& ref, const Eigen::Vector& impulse)
{
    applyImpulse(getR(ref), impulse);
}

void RigidBody::applyForce(SimulationPointRef& ref, const Eigen::Vector& force)
{
    applyForce(getR(ref), force);
}

void RigidBody::applyImpulse(const Eigen::Vector3d& r, const Eigen::Vector3d& p)
{
    if (mStatic)
        return;

    mV += mMassInv * p;
    mOmega += mInertiaInv * (r.cross(p));
}

void RigidBody::applyOrientationImpulse(const Vector& l)
{
    mOmega += mInertiaInv * l;
}

void RigidBody::applyForce(const Vector3d& r, const Vector3d& force)
{
    if (mStatic)
        return;

    mForceExt += force;
//    mTorqueExt += (mQ.toRotationMatrix() * r).cross(force);
    mTorqueExt += (r).cross(force);
    //    std::cout << r.transpose() << " ||| " << (mQ.toRotationMatrix() * r).transpose() << "\n";
}

void RigidBody::applyTorque(const Vector& torque)
{
    mTorqueExt += torque;
}

void RigidBody::applyDamping()
{
    mV *= (1 - mTranslationalDamping);
    mOmega *= (1 - mRotationalDamping);
}

Vector RigidBody::getR(SimulationPointRef& pointRef)
{
    switch(pointRef.getGeometricType())
    {
    case GeometricPointRef::Type::POLYGON_VECTOR:
        return mQ * static_cast<PolygonVectorRef*>(
                    pointRef.getGeometricPointRef())->getR();
    case GeometricPointRef::Type::GEOMETRIC_VERTEX:
        return mQ * mPolygon->getPositionBS(static_cast<GeometricVertexRef*>(
                                                pointRef.getGeometricPointRef())->getIndex());
    }
    return Vector::Zero();
}

Matrix3d RigidBody::calculateK(const Vector& rA, const Vector& rB)
{
    if (mStatic)
        return Matrix3d::Zero();

    Eigen::Matrix3d rA_cross;
    rA_cross << 0, -rA(2), rA(1),
            rA(2), 0, -rA(0),
            -rA(1), rA(0), 0;

    Eigen::Matrix3d rB_cross;
    rB_cross << 0, -rB(2), rB(1),
            rB(2), 0, -rB(0),
            -rB(1), rB(0), 0;

    return mMassInv * Eigen::Matrix3d::Identity() -
            rA_cross * mInertiaInv * rB_cross;
}

Matrix3d RigidBody::calculateL()
{
    return mInertiaInv;
}

void RigidBody::accept(SimulationObjectVisitor& visitor)
{
    visitor.visit(*this);
}

void RigidBody::updateGeometricData(bool notifyListeners)
{
    Eigen::Affine3d transform;
    transform.linear() = mQ.toRotationMatrix();
    transform.translation() = mX;
    mPolygon->setTransform(transform);
//    mPolygon->updatePositions();
    mPolygon->update(true, false, notifyListeners);
//    mPolygon->getTransform().setIdentity();
//    mPolygon->getTransform().rotate(mQ);
    //    mPolygon->getTransform().translate(mX);
}

Eigen::Vector& RigidBody::getPosition(size_t /*id*/)
{
    return mX;
    // return mPositions[id];
}

void RigidBody::setPosition(Eigen::Vector v, ID /*id*/)
{
    mX = v;
    // mPositions[id] = v;
}

void RigidBody::addToPosition(Eigen::Vector v, ID /*id*/)
{
    mX += v;
    //mPositions[id] += v;
}

size_t RigidBody::getSize()
{
    return 1;
    // return mPositions.size();
}

GeometricData* RigidBody::getGeometricData()
{
    return mPolygon.get();
}
