#ifndef SELECTIONRECTANGLE_H
#define SELECTIONRECTANGLE_H

#include <data_structures/DataStructures.h>

class ViewFrustum;

class SelectionRectangle
{
public:
    SelectionRectangle();

    // Getters
        int const& getXStart() const;
        int const& getYStart() const;
        int const& getXEnd() const;
        int const& getYEnd() const;

        int& getXStart();
        int& getYStart();
        int& getXEnd();
        int& getYEnd();

    // Setters
        void setRectangle(
                int xStart,
                int yStart,
                int xEnd,
                int yEnd);
        void setActive(bool active);

    // Getters
        bool isActive();

    // Tests inside methods
        // Tests if the given vertex is inside
        // the given view frustum. Returns true
        // if it is.
        bool testVertex(
                Eigen::Vector& v,
                ViewFrustum* viewFrustum);

protected:
    int mXStart;
    int mYStart;
    int mXEnd;
    int mYEnd;
    bool mActive;
};

#endif // SELECTIONRECTANGLE_H
