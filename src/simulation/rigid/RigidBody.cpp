#include "RigidBody.h"

#include <simulation/SimulationObjectVisitor.h>

#include <math/MathUtils.h>

#include <scene/data/GeometricData.h>

#include <scene/data/geometric/Polygon.h>

#include <iostream>

#include <scene/data/references/GeometricPointRef.h>
#include <scene/data/references/GeometricPointRefVisitor.h>
#include <scene/data/references/GeometricVertexRef.h>
#include <scene/data/references/PolygonVectorRef.h>

#include <simulation/references/SimulationPointRef.h>

using namespace Eigen;

RigidBody::RigidBody(
        Domain* domain,
        std::shared_ptr<Polygon> polygon,
        Vectors& positions,
        double mass)
    : SimulationObject(domain)
    , mPolygon(polygon)
    , mPositions(positions)
    , mMass(mass)
    , mStatic(false)
{
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
    // v_{i+1} = v_i + h * M^{-1} f_i
    mV = mV + timeStep * (1.0 / mMass) * mForceExt;

    // \omega_{i+1} = \omega_i + h * I^{-1} (\tau_{ext} - (\omega_i \times (I \omega_i)))
    mOmega = mOmega + timeStep * mInertiaInv * (mTorqueExt - (mOmega.cross(mInertia * mOmega)));

    // 2.) integrate positions/rotations
    integratePositions(timeStep);

    update();
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

    Eigen::Matrix3d rot = mQ.toRotationMatrix();
    Eigen::Matrix3d inertiaInv = rot * mInertiaInvBS * rot.transpose();

//    Eigen::Matrix3d inertiaInv = rot.transpose() * mInertiaInvBS * rot;
    mV = mV + 1 / mMass * p;
    mOmega = mOmega + inertiaInv * (r.cross(p));
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
    class GetRVisitor : public GeometricPointRefVisitor
    {
    public:
        GetRVisitor(RigidBody& _rb)
            : rb(_rb)
        {
        }

        virtual void visit(GeometricVertexRef& ref)
        {
            // this assumes that the geometric datas initial positions
            // (which remain unchanged for rigid bodies) have their origin
            // in the center of mass.
            r = rb.getOrientation().toRotationMatrix() *
                    rb.getPolygon()->getPositionBS(ref.getIndex());
        }

        virtual void visit(PolygonVectorRef& ref)
        {
            r = rb.getOrientation().toRotationMatrix() * ref.getR();
        }

        RigidBody& rb;
        Eigen::Vector r;
    } visitor(*this);

    pointRef.getGeometricPointRef()->accept(visitor);
    return visitor.r;
}

Vector RigidBody::calculateSpeedAt(const Vector& r)
{
    if (mStatic)
        return Vector::Zero();

    return mV + mOmega.cross(r);
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

    return 1 / mMass * Eigen::Matrix3d::Identity() -
            rA_cross * mInertiaInv * rB_cross;
}

Matrix3d RigidBody::calculateL()
{
    return mInertiaInv;
}

void RigidBody::updateGeometricData()
{
    Eigen::Affine3d transform;
    transform.setIdentity();
    transform.translate(mX);
    transform.rotate(mQ);
    mPolygon->setTransform(transform);
//    mPolygon->updatePositions();
    mPolygon->update();
//    mPolygon->getTransform().setIdentity();
//    mPolygon->getTransform().rotate(mQ);
    //    mPolygon->getTransform().translate(mX);
}

void RigidBody::setTranslationalDamping(double translationalDamping)
{
    mTranslationalDamping = translationalDamping;
}

void RigidBody::setRotationalDamping(double rotationalDamping)
{
    mRotationalDamping = rotationalDamping;
}

void RigidBody::setMass(double mass)
{
    mMass = mass;
}

void RigidBody::setStatic(bool s)
{
    mStatic = s;
}

const Vector3d& RigidBody::getCenterOfMass() const
{
    return mX;
}

const Quaterniond& RigidBody::getOrientation() const
{
    return mQ;
}

const Quaterniond& RigidBody::getOrientationPrevious() const
{
    return mQOld;
}

const Eigen::Vector& RigidBody::getOrientationVelocity() const
{
    return mOmega;
}

const Matrix3d& RigidBody::getInertiaTensor() const
{
    return mInertiaBS;
}

const Matrix3d& RigidBody::getInveresInertiaTensor() const
{
    return mInertiaInvBS;
}

const Matrix3d& RigidBody::getInertiaTensorWS() const
{
    return mInertia;
}

const Matrix3d& RigidBody::getInverseInertiaTensorWS() const
{
    return mInertiaInv;
}

const Vector& RigidBody::getPosition() const
{
    return mX;
}

const Vector& RigidBody::getPositionPrevious() const
{
    return mXOld;
}

std::shared_ptr<Polygon> RigidBody::getPolygon()
{
    return mPolygon;
}

double RigidBody::getTranslationalDamping() const
{
    return mTranslationalDamping;
}

double RigidBody::getRotationalDamping() const
{
    return mRotationalDamping;
}

double RigidBody::getMass() const
{
    return mMass;
}

bool RigidBody::isStatic() const
{
    return mStatic;
}

SimulationObject::Type RigidBody::getType() const
{
    return RIGID_BODY;
}

void RigidBody::accept(SimulationObjectVisitor& visitor)
{
    visitor.visit(*this);
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
