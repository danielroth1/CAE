#include "RenderThread.h"

#include <QOffscreenSurface>
#include <QOpenGLContext>
#include <glwidget.h>

#include <multi_threading/TimeStepper.h>


RenderThread::RenderThread(GLWidget* widget)
    : mWidget(widget)
{
    // add a TimeStepper that allows FPS control
    setTimeStepper(std::make_shared<TimeStepper>(20));
}

void RenderThread::initialization()
{
//    mSurface = new QOffscreenSurface();

//    if (!mContext)
//    {
//        QOpenGLContext* current = mWidget->context();
//        mContext = new QOpenGLContext();
//        mContext->setFormat(current->format());
//        mContext->setShareContext(current);
//        mContext->create();
//        mContext->moveToThread()

//    }
}

void RenderThread::beforeStep()
{
//    if (mWidget->ise)
//    mWidget->makeCurrent();
}

void RenderThread::step()
{
    mWidget->update();
}
