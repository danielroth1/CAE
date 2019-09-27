#include "glwidget.h"

#include <times/timing.h>


#include <GL/glut.h>
#include <GL/glu.h>
#include <math.h>
#include <QKeyEvent>

#include "constants.h"

#include <ui/KeyManager.h>
#include <ui/KeyManager.h>
#include <ui/UIControl.h>
#include <rendering/Renderer.h>

#include <iostream>
#include <QDebug>
#include <QTimer>


GLWidget::GLWidget(QWidget *parent)
    : QOpenGLWidget(parent)
{
    setDefaults();
    mRenderer = nullptr;
}

GLWidget::~GLWidget()
{

}

void GLWidget::setDefaults()
{
    mMouseSensitivy = 1.0f;
    mMovementSpeed = 10.0f;
    mMousePos = QPointF(0.0, 0.0);
    mAngle = QPointF(0.0, -15.0);
    mCameraPos = QVector3D(0.0, 1.5f, 5.5f);
    angleToDir();
    mRenderer = nullptr;
}

QVector3D GLWidget::getCameraPos() const
{
    return mCameraPos;
}

QVector3D GLWidget::getCameraDir() const
{
    return mCameraDir;
}

void GLWidget::updateCameraSpeedAfterKeyPress()
{
    KeyManager* km = KeyManager::instance();

    QVector3D cameraVel;
    cameraVel.setX(0);
    cameraVel.setY(0);
    cameraVel.setZ(0);

    if (km->isKeyDown(Qt::Key::Key_W))
    {
        cameraVel += mMovementSpeed * mCameraDir;
    }

    if (km->isKeyDown(Qt::Key::Key_S))
    {
        cameraVel -= mMovementSpeed * mCameraDir;
    }

    if (km->isKeyDown(Qt::Key::Key_A))
    {
        QVector3D ortho(-mCameraDir.z(), 0.0, mCameraDir.x());
        ortho.normalize();
        cameraVel -= mMovementSpeed * ortho;
    }

    if (km->isKeyDown(Qt::Key::Key_D))
    {
        QVector3D ortho(-mCameraDir.z(), 0.0, mCameraDir.x());
        ortho.normalize();
        cameraVel += mMovementSpeed * ortho;
    }

    if (km->isKeyDown(Qt::Key::Key_Q))
    {
        cameraVel[1] += mMovementSpeed;
    }

    if (km->isKeyDown(Qt::Key::Key_E))
    {
        cameraVel[1] -= mMovementSpeed;
    }

    mCameraVel = cameraVel;
}

void GLWidget::setUIControl(UIControl *uiControl)
{
    mUiControl = uiControl;
}

void GLWidget::setRenderer(Renderer* renderer)
{
    mRenderer = renderer;
}

void GLWidget::processTimedEvent(const int /*x*/)
{

}

void GLWidget::initializeGL()
{
    //initializeOpenGLFunctions();

    // black screen
    glClearColor(0,0,0,0);
    // enable depth buffer
    glEnable(GL_DEPTH_TEST);
    // set shading model
    glShadeModel(GL_SMOOTH);

//    SimulationControl* sc = new SimulationControl(this);
//    //sc.handleFileInput(argc, argv);

//    setDrawable(sc->initialize()->getRenderSimulation());
    // TODO: send redraw signal to UI every time simulation step finished

    mTime.start();
    mMillisInThisSecond = 0;
    mFramesInThisSecond = 0;
}

void GLWidget::resizeGL(int width, int height)
{
    //glClear(GL_COLOR_BUFFER_BIT);
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(65.0, static_cast<double>(width) / height,
                   0.1, 10000);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void GLWidget::keyPressEvent(QKeyEvent* keyEvent)
{
    QWidget::keyPressEvent(keyEvent);
}

void GLWidget::keyReleaseEvent(QKeyEvent* keyEvent)
{
    QWidget::keyReleaseEvent(keyEvent);
}

void GLWidget::mousePressEvent(QMouseEvent* event)
{
    mMousePos = event->localPos();
    mUiControl->mousePressEvent(event);

    QWidget::mousePressEvent(event);
}

void GLWidget::mouseReleaseEvent(QMouseEvent* event)
{
    mUiControl->mouseReleaseEvent(event);

    QWidget::mouseReleaseEvent(event);
}

void GLWidget::mouseMoveEvent(QMouseEvent* event)
{
    qreal x = event->localPos().x();
    qreal y = event->localPos().y();
    if (event->buttons() == Qt::LeftButton)
    {
        mAngle.rx() = fmod(mAngle.x() + (x - mMousePos.x() * static_cast<double>(mMouseSensitivy)), 360.0);
        mAngle.ry() -= (y - mMousePos.y()) * static_cast<double>(mMouseSensitivy);
        mAngle.ry() = max(-70.0, min(70.0, mAngle.y()));

        angleToDir();
    }
    if (event->buttons() == Qt::RightButton)
    {
        //cameraPos[2] -= 0.2*(y - mousePos.y()) * mouseSensitivy;
    }
    if (event->buttons() == Qt::MiddleButton)
    {
        mCameraPos[0] += static_cast<float>(0.2 * (x - mMousePos.x()) * static_cast<double>(mMouseSensitivy));
        mCameraPos[1] -= static_cast<float>(0.2 * (y - mMousePos.y()) * static_cast<double>(mMouseSensitivy));
    }
    mMousePos = event->localPos();
    mUiControl->mouseMoveEvent(event);

    QWidget::mouseMoveEvent(event);
}

void GLWidget::paintGL()
{
    START_TIMING_RENDERING("GLWidget::paintGL()");

    updateCameraPos(0.01f);

    mUiControl->handlePreRenderingStep();
    if (mRenderer)
    {
        // clear and set camera
        START_TIMING_RENDERING("GLWidget::clear(), gluLookAt() usw.")

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glLoadIdentity();
        QVector3D cameraLookAt = mCameraPos + mCameraDir;
        gluLookAt(static_cast<double>(mCameraPos.x()),
                  static_cast<double>(mCameraPos.y()),
                  static_cast<double>(mCameraPos.z()),
                  static_cast<double>(cameraLookAt.x()),
                  static_cast<double>(cameraLookAt.y()),
                  static_cast<double>(cameraLookAt.z()),
                  0.0, 1.0, 0.0);
        mUiControl->handleProjectionUpdated();
        STOP_TIMING_RENDERING;
//        const double w = width();
//        const double h = height();
    //    std::cout << w << ", " << h << "\n";
    //    // translate to centerPos
    //    glTranslatef(cameraPos.x(), cameraPos.y(), cameraPos.z());
    //    // rotate scene
    //    glRotatef(angle.x(),0.0f,1.0f,0.0f);
    //    glRotatef(angle.y(),1.0f,0.0f,0.0f);

    //    glutWireTeapot(0.6f);

        //glPolygonMode( GL_FRONT_AND_BACK, GL_FILL);
    //    glEnable(GL_LIGHTING);
    //    glColor3f(0.2,0.5,0.8);
    //    glutSolidTeapot(0.6f);

    //    glPolygonMode( GL_FRONT_AND_BACK, GL_FILL);
    //    glBegin(GL_TRIANGLES);
    //        glVertex3f(0.0f, 0.0f, 0.0f);
    //        glVertex3f(1.0f, 1.0f, 0.0f);
    //        glVertex3f(0.0f, 0.0f, 1.0f);
    //    glEnd();

        mRenderer->draw();

        mFramesInThisSecond++;

        mMillisInThisSecond = mTime.elapsed();
        if (mMillisInThisSecond >= 1000)
        {
            std::cout << "fps = " << mFramesInThisSecond << "\n";

            mTime.restart();
            mMillisInThisSecond = 0;
            mFramesInThisSecond = 0;
        }
    }
//    START_TIMING_RENDERING("GLWidget::glFinish()")
//    glFinish();
//    STOP_TIMING_RENDERING

    STOP_TIMING_RENDERING
}

void GLWidget::angleToDir()
{
    mCameraDir[0] =  sin(mAngle.x()*M_RadToDeg) * cos(mAngle.y()*M_RadToDeg);
    mCameraDir[2] = -cos(mAngle.x()*M_RadToDeg) * cos(mAngle.y()*M_RadToDeg);
    mCameraDir[1] = max(0.0f,min(sqrtf(1.0f - mCameraDir.x()*mCameraDir.x() -  mCameraDir.z()* mCameraDir.z()),1.0f));
    if (mAngle.y() < 0)
        mCameraDir[1] = -mCameraDir[1];
}

void GLWidget::updateCameraPos(float stepSize)
{
    mCameraPos += stepSize * mCameraVel;
}
