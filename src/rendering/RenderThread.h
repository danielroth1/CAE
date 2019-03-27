#ifndef RENDERTHREAD_H
#define RENDERTHREAD_H

#include <multi_threading/StepperThread.h>


class GLWidget;
class QOffscreenSurface;
class QOpenGLContext;


class RenderThread : public StepperThread
{
public:
    RenderThread(GLWidget* widget);

    // StepperThread interface
public:
    virtual void initialization();
    virtual void beforeStep();
    virtual void step();

private:

//    QOpenGLContext* mContext;
//    QOffscreenSurface* mSurface;

    GLWidget* mWidget;
};

#endif // RENDERTHREAD_H
