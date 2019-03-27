#ifndef POINTREFCOLLECTION_H
#define POINTREFCOLLECTION_H

#include <data_structures/DataStructures.h>

// A collection of point references.
// Access points from the collection with getPoint(ID).
// Get size of collection with getSize().
// Holds a collection of T objects.
// This class is kinda obsolet.
template <class T, class P>
class PointRefCollection
{
public:
    PointRefCollection();

//    // Add a point for object.
//    virtual void addPoint(T object, ID index) = 0;

//    // Remove point of object at index.
//    // \return false if there was no object or no index.
//    virtual bool removePoint(T object, ID index) = 0;

//    // Remove all points of object.
//    // \return false if there was no object to remove
//    virtual bool removePoints(T object) = 0;

    virtual ID getSize() = 0;

};

#include "PointRefCollection.cpp"

#endif // POINTREFCOLLECTION_H
