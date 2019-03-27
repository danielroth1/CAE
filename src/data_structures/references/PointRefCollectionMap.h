#ifndef POINTREFCOLLECTIONMAP_H
#define POINTREFCOLLECTIONMAP_H

#include "PointRefCollection.h"
#include <map>
#include <vector>


// Does not implement getPoint() because this requires knowledge about
// class T how to access that point from a given index.
template <class T, class P>
class PointRefCollectionMap : public PointRefCollection<T, P>
{
public:
    PointRefCollectionMap();

    // PointRefCollection interface
public:
    // Add a point for object.
    void addPoint(T object, ID index);

    void addPoints(T object, const std::vector<ID>& indices);

    // Remove point of object at index.
    // \return false if there was no object or no index.
    bool removePoint(T object, ID index);

    // Remove all points of object.
    // \return false if there was no object to remove
    bool removePoints(T object);

    virtual ID getSize() override;

    // Getters
        ID getPointIndex(T object, ID index);

        std::vector<ID> getPointIndices(T object);

        virtual P getPoint(T object, ID index) = 0;

    typename std::map<T, std::vector<ID>>::iterator begin();
    typename std::map<T, std::vector<ID>>::iterator end();

protected:
    std::map<T, std::vector<ID>> mMap;
};

#include "PointRefCollectionMap.cpp"

#endif // POINTREFCOLLECTIONMAP_H
