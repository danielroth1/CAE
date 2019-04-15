#ifndef RIGIDBODY_H
#define RIGIDBODY_H

#include <data_structures/DataStructures.h>
#include <simulation/SimulationObject.h>

class Domain;
class Polygon;
class SimulationPointRef;

// Rigid body does not depend on Geometric Data because
// it should be as independent as possible.
class RigidBody : public SimulationObject
{
public:
    // It is assumed that each vertex has a constant mass.
    // From the vertex positions the inertia tensor is
    // calculated.
    // \param positions - of the to be approximated polygon
    // \param mass - the mass of each vertex
    RigidBody(Domain* domain,
              std::shared_ptr<Polygon> polygon,
              Vectors& positions,
              double mass);

    // Calculates all relevant data w.r.t. to body space coordinates.
    // Call this method each time position x or rotation q changes.
    void update();

    void prepareNewStep();

    // Explicit Euler time integration
    // 1.) integrate velocities:
    // v_{i+1} = v_i + h * M^{-1} f_i
    // \omega_{i+1} = \omega_i + I^{-1} (\tau_{ext} - (\omega \times (J\omega)))
    //
    // 2.) integrate positions/rotations
    // x_{i+1} = x_i + h * v_{i+1}
    // q_{i+1} = q_i + h * 0.5 * toQuat(\omega_{i+1}) \otimes q_i
    void solveExplicit(double timeStep);

    virtual void integratePositions(double timeStep) override;

    // The positions of the current time step can be reverted to
    // the previous one. The reversion of only a single time step
    // is possible. Do not call this method twice wihtout calling
    // a solveExplicit() inbetween.
    virtual void revertPositions() override;

    // Rigid Dynamics Methods
        virtual void applyImpulse(
            SimulationPointRef& ref,
            const Eigen::Vector& impulse) override;

        virtual void applyForce(
            SimulationPointRef& ref,
            const Eigen::Vector& force) override;

        // \param r - the point on the body where the impulse is applied on
        // \param p - the impulse vector.
        void applyImpulse(const Eigen::Vector& r, const Eigen::Vector& p);

        void applyOrientationImpulse(const Eigen::Vector& l);

        void applyForce(const Eigen::Vector3d& r, const Eigen::Vector3d& force);

        void applyDamping();

    Eigen::Vector getR(SimulationPointRef& pointRef);

    // Collision Resolution


        // Calculates and returns the speed of a point on this rigid
        // at r. r points from the center of the rigid to the point.
        Eigen::Vector calculateSpeedAt(const Eigen::Vector& r);

        Eigen::Matrix3d calculateK(
                const Eigen::Vector& rA,
                const Eigen::Vector& rB);

        Eigen::Matrix3d calculateL();

    // Scene Handling
        // Updates the geometric datas transformation matrix.
        // Call this to inform the geometric data about changes, e.g.
        // at the end of a simulation step.
        void updateGeometricData();

    // Setters
        void setTranslationalDamping(double translationalDamping);
        void setRotationalDamping(double rotationalDamping);

        void setStatic(bool s);

    // Getters
        // Returns the center of mass x.
        const Eigen::Vector3d& getCenterOfMass() const;

        // Returns the orientation q.
        const Eigen::Quaterniond& getOrientation() const;
        const Eigen::Vector& getOrientationVelocity() const;

        // Returns the 3x3 inertia tensor I.
        const Eigen::Matrix3d& getInertiaTensor() const;
        const Eigen::Matrix3d& getInveresInertiaTensor() const;

        const Eigen::Matrix3d& getInertiaTensorWS() const;
        const Eigen::Matrix3d& getInverseInertiaTensorWS() const;

        const Eigen::Vector& getPosition() const;
        const Eigen::Vector& getPositionPrevious() const;

        std::shared_ptr<Polygon> getPolygon();

        double getTranslationalDamping() const;
        double getRotationalDamping() const;

        double getMass() const;

        bool isStatic() const;

    // SimulationObject interface
public:
    virtual Type getType() const override;
    virtual void accept(SimulationObjectVisitor& visitor) override;
    virtual Eigen::Vector& getPosition(size_t id) override;
    virtual void setPosition(Eigen::Vector v, ID id) override;
    virtual void addToPosition(Eigen::Vector v, ID id) override;
    virtual size_t getSize() override;
    virtual GeometricData* getGeometricData() override;

private:

    // Geometric Data
    std::shared_ptr<Polygon> mPolygon;
    Vectors& mPositions;

    // Center of mass
    Eigen::Vector3d mX;
    Eigen::Vector3d mV;
    Eigen::Vector3d mForceExt;

    // Orientation
    Eigen::Quaterniond mQ;
    Eigen::Vector3d mOmega;
    Eigen::Vector3d mTorqueExt;

    // Old Position/ Orientation
    Eigen::Vector3d mXOld;
    Eigen::Quaterniond mQOld;

    // Inertia tensor
    Eigen::Matrix3d mInertiaBS;
    Eigen::Matrix3d mInertiaInvBS;

    // Inverse inertia tensor
    Eigen::Matrix3d mInertia;
    Eigen::Matrix3d mInertiaInv;

    double mMass;

    double mTranslationalDamping;
    double mRotationalDamping;

    bool mStatic;


};

#endif // RIGIDBODY_H
