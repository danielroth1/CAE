#ifndef POINTREFCOLLECTIONVECTOR_H
#define POINTREFCOLLECTIONVECTOR_H


#include <data_structures/DataStructures.h>
#include "PointRefCollection.h"


// Does not implement getPoint() because this requires knowledge about
// class T how to access that point from a given index.
template <class T, class P>
class PointRefCollectionVector : PointRefCollection<T, P>
{
public:
    PointRefCollectionVector(T object);
    PointRefCollectionVector(T object, const std::vector<ID>& indices);
    // TODO: adapt function signatures

    // Delegated vector methods

        // Add a point for object.
        void addPoint(ID index);

        void addPoints(const std::vector<ID>& indices);

        // Remove point of object at index.
        // \return false if there was no object or no index.
        bool removePoint(ID index);

        // Remove all points of object.
        // \return false if there was no object to remove
        void removePoints();

        virtual ID getSize() override;

        ID getPointIndex(ID index);

        virtual P getPoint(ID index) = 0;

    T getObject();

    typename std::vector<ID>::iterator begin();
    typename std::vector<ID>::iterator end();

protected:
    T mObject;
    std::vector<ID> mIndices;
};

#include "PointRefCollectionVector.cpp"

#endif // POINTREFCOLLECTIONVECTOR_H
