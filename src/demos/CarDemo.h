#ifndef CARDEMO_H
#define CARDEMO_H

#include <data_structures/DataStructures.h>
#include <modules/demo_loader/Demo.h>
#include <scene/scene_graph/SGCore.h>

#include <string>

class ApplicationControl;
class SGControl;
class SimulationPointRef;

// A simple car with four tires that are connected to a rigid by
// a line joints. They hold the tire on the line that goes downwards from
// the rigid. A spring damper holds it in place perpendicular to that line.
//
// The car drives in a wall of rigid boxes.
//
class CarDemo : public Demo
{
public:
    CarDemo(ApplicationControl& ac);

    // Demo interface
public:
    virtual std::string getName();
    virtual void load();
    virtual void unload();

private:
    SGChildrenNode* createCar(
            Eigen::Affine3d transformation,
            double width,
            double length,
            double height,
            double tireWidth,
            double springLength,
            double cSpring,
            double cDamping);

    void createTire(
            SGChildrenNode* parent,
            std::string name,
            Eigen::Affine3d transformation,
            const SimulationPointRef& target,
            double tireWidth, double springLength,
            double cSpring, double cDamping);

    ApplicationControl& mAc;
    SGControl* mSg;
};

#endif // CARDEMO_H
