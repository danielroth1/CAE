#include "SelectionRectangle.h"

#include <GL/glu.h>
#include <rendering/ViewFrustum.h>

SelectionRectangle::SelectionRectangle()
    : mXStart(0)
    , mYStart(0)
    , mXEnd(0)
    , mYEnd(0)
    , mActive(false)
{

}

int const& SelectionRectangle::getXStart() const
{
    return mXStart;
}

int const& SelectionRectangle::getYStart() const
{
    return mYStart;
}

int const& SelectionRectangle::getXEnd() const
{
    return mXEnd;
}

int const& SelectionRectangle::getYEnd() const
{
    return mYEnd;
}

int& SelectionRectangle::getXStart()
{
    return mXStart;
}

int& SelectionRectangle::getYStart()
{
    return mYStart;
}

int& SelectionRectangle::getXEnd()
{
    return mXEnd;
}

int& SelectionRectangle::getYEnd()
{
    return mYEnd;
}

void SelectionRectangle::setRectangle(
        int xStart,
        int yStart,
        int xEnd,
        int yEnd)
{
    mXStart = xStart;
    mYStart = yStart;
    mXEnd = xEnd;
    mYEnd = yEnd;
}

void SelectionRectangle::setActive(bool active)
{
    mActive = active;
}

bool SelectionRectangle::isActive()
{
    return mActive;
}

bool SelectionRectangle::testVertex(
        Eigen::Vector& v,
        ViewFrustum* viewFrustum)
{
    double* PR = viewFrustum->getProjection();
    double* MV = viewFrustum->getModelView();
    int* VP = viewFrustum->getViewPort();

    double x_min = std::min(getXStart(), getXEnd());
    double x_max = std::max(getXStart(), getXEnd());
    double y_min = std::min(getYStart(), getYEnd());
    double y_max = std::max(getYStart(), getYEnd());

//    // convert qt to gl coordinates
//    x_min = x_min - VP[0];
//    y_min = y_min - VP[1];
//    x_max = x_max - VP[2];
//    y_max = y_max - VP[3];

//    // revert y-axis
//    y_min = VP[3] - y_min;
//    y_max = VP[3] - y_max;

    GLdouble x;
    GLdouble y;
    GLdouble z;

    GLdouble* x_ptr = &x;
    GLdouble* y_ptr = &y;
    GLdouble* z_ptr = &z;
    gluProject(v.x(), v.y(), v.z(), MV, PR, VP, x_ptr, y_ptr, z_ptr);

    return x >= x_min && x <= x_max &&
            y >= y_min && y <= y_max;
}
