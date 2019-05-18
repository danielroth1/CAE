#include "BSWSVectors.h"

using namespace Eigen;

BSWSVectors::BSWSVectors()
{
    mInitialized = false;
}

BSWSVectors::BSWSVectors(size_t size)
{
    Vectors vectors;
    vectors.resize(size);
    for (size_t i = 0; i < size; ++i)
    {
        vectors[i] = Eigen::Vector::Zero();
    }
    initializeFromWorldSpace(vectors);
}

BSWSVectors::BSWSVectors(
        const Vectors& vectorsWS)
{
    initializeFromWorldSpace(vectorsWS);
}

BSWSVectors::BSWSVectors(
        Vectors* vectorsBS,
        const Affine3d& transform)
{
    initializeFromBodySpace(vectorsBS, transform);
}

BSWSVectors::BSWSVectors(const BSWSVectors& posData)
{
    if (!posData.mInitialized)
    {
        mInitialized = false;
        return;
    }

    switch(posData.mType)
    {
    case BODY_SPACE:
        initializeFromBodySpace(posData.mVectorsBS, posData.mTransform);
        break;
    case WORLD_SPACE:
        initializeFromWorldSpace(posData.mVectorsWS);
        break;
    }
}

// go to BSWSVectors
void BSWSVectors::update()
{
    if (!mInitialized)
        return;

    switch (mType)
    {
    case BODY_SPACE:
        updateWorldSpace();
        break;
    case WORLD_SPACE:
        // do nothing.
        // The body space can only be synchronized with
        // a center vertex. In general it is not necessary
        // to keep the body space synchronized. The relevant
        // data are the world space positions and they are
        // already given.
        //updateBodySpace();
        break;
    }
}

BSWSVectors::Type BSWSVectors::getType()
{
    return mType;
}

size_t BSWSVectors::getSize()
{
    return mVectorsWS.size();
}

Vectors& BSWSVectors::getVectors()
{
    return mVectorsWS;
}

const Vectors& BSWSVectors::getVectors() const
{
    return mVectorsWS;
}

Vector& BSWSVectors::getVector(ID index)
{
    return mVectorsWS[index];
}

const Vector& BSWSVectors::getVector(ID index) const
{
    return mVectorsWS[index];
}

Affine3d& BSWSVectors::getTransform()
{
    return mTransform;
}

Vectors& BSWSVectors::getVectorsBS()
{
    return *mVectorsBS;
}

Vector& BSWSVectors::getVectorBS(ID index)
{
    return (*mVectorsBS)[index];
}

void BSWSVectors::setTransform(const Affine3d& transform)
{
    mTransform = transform;
}

// go to BSWSVectors without center ( = (0, 0, 0) )
// setting the center should be done only in SharedPolygonData
void BSWSVectors::changeRepresentationToBS(
        Vectors* vectorsBS,
        const Eigen::Affine3d& transform)
{
    if (!mInitialized)
        return;

    // one may only change the representation type to
    // body space if it was in world space before because
    // then it is ensured that the world space representation
    // is valid.
    if (mType == WORLD_SPACE)
    {
        mVectorsBS = vectorsBS;
        mVectorsWS = *vectorsBS;
        mTransform = transform;
        mType = BODY_SPACE;

        updateWorldSpace();
    }
}

// go to BSWSVectors
void BSWSVectors::changeRepresentationToWS()
{
    if (!mInitialized)
        return;

    // one may only change the representation type to
    // world space if it was in body space before because
    // then it is ensured that the body space representation
    // is valid.
    if (mType == BODY_SPACE)
    {
        updateWorldSpace();
        mType = WORLD_SPACE;
    }
}

// go to BSWSVectors
void BSWSVectors::translate(const Vector& t)
{
    if (!mInitialized)
        return;

    switch(mType)
    {
    case BODY_SPACE:
        mTransform.translate(t);
        break;
    case WORLD_SPACE:
        for (Vector& v : mVectorsWS)
        {
            v += t;
        }
    }
}

void BSWSVectors::transform(const Affine3d& transform)
{
    if (!mInitialized)
        return;

    switch(mType)
    {
    case BODY_SPACE:
        mTransform = mTransform * transform;
        break;
    case WORLD_SPACE:
        for (Vector& v : mVectorsWS)
        {
            v = transform * v;
        }
    }
}

void BSWSVectors::updateWorldSpace()
{
    if (!mInitialized)
        return;

    for (size_t i = 0; i < mVectorsWS.size(); ++i)
    {
        mVectorsWS[i] = mTransform * (*mVectorsBS)[i];
    }
}

void BSWSVectors::initializeFromWorldSpace(Vectors vectorsWS)
{
    mVectorsWS = vectorsWS;
    mVectorsBS = nullptr;
    mTransform.setIdentity();
    mType = WORLD_SPACE;
    mInitialized = true;
}

void BSWSVectors::initializeFromBodySpace(
        Vectors* vectorsBS,
        const Affine3d& transform)
{
    mVectorsBS = vectorsBS;
    mTransform = transform;

    // world space positions is simply equal body Ogespace
    // positions.
    mVectorsWS = *vectorsBS;

    mType = BODY_SPACE;
    mInitialized = true;
}
