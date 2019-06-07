#pragma once

#ifndef BUFFEREDDATA_CPP
#define BUFFEREDDATA_CPP

#include "BufferedData.h"
#include <GL/glew.h>

#include <rendering/RendererUtils.h>

template <class T, class NT, unsigned int NE>
BufferedData<T, NT, NE>::BufferedData(
        GLenum target,
        GLenum usage)
    : mTarget(target)
    , mUsage(usage)
{
    mVbo = 0;
    mInitialized = false;
    mDataChanged = true;
}

template<class T, class NT, unsigned int NE>
BufferedData<T, NT, NE>::BufferedData(BufferedData<T, NT, NE>& bd)
    : mData(bd.mData)
    , mVbo(bd.mVbo)
    , mTarget(bd.mTarget)
    , mUsage(bd.mUsage)
    , mInitialized(bd.mInitialized)
    , mDataChanged(bd.mDataChanged)
{
}

template<class T, class NT, unsigned int NE>
void BufferedData<T, NT, NE>::initialize()
{
    cleanup();
    createBuffer();
    mInitialized = true;
}

template<class T, class NT, unsigned int NE>
void BufferedData<T, NT, NE>::cleanup()
{
    if (mVbo != 0)
        glDeleteBuffers(1, &mVbo);
    mVbo = 0;
    mInitialized = false;
}

template<class T, class NT, unsigned int NE>
void BufferedData<T, NT, NE>::bindBuffer()
{
    glBindBuffer(mTarget, mVbo);
}

template<class T, class NT, unsigned int NE>
void BufferedData<T, NT, NE>::createBuffer()
{
    if (mVbo == 0)
        mVbo = RendererUtils::createVBO();

    mInitialized = true;
}

template<class T, class NT, unsigned int NE>
void BufferedData<T, NT, NE>::refreshBuffer()
{
    if (mDataChanged && mInitialized)
    {
        auto data = mData.lock();
        if (mData.unsafe().size() > 0)
            RendererUtils::refreshVBO(
                        mVbo,
                        mData.unsafe().data(),
                        NE * sizeof(NT) * mData.unsafe().size(),
                        mTarget,
                        mUsage);

        mDataChanged = false;
    }
}

template<class T, class NT, unsigned int NE>
Monitor<std::vector<T>>& BufferedData<T, NT, NE>::getData()
{
    return mData;
}

template<class T, class NT, unsigned int NE>
bool BufferedData<T, NT, NE>::isInitialized() const
{
    return mInitialized;
}

template<class T, class NT, unsigned int NE>
void BufferedData<T, NT, NE>::setDataChanged(bool dataChanged)
{
    mDataChanged = dataChanged;
}

template<class T, class NT, unsigned int NE>
bool BufferedData<T, NT, NE>::isDataChanged() const
{
    return mDataChanged;
}

#endif // BUFFEREDDATA_CPP

