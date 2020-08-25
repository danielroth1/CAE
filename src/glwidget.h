#ifndef GLWIDGET_H
#define GLWIDGET_H

// Includes
#include <Eigen/Core>
#include <SimulationControl.h>

//#include <QOpenGLFunctions>
#include <QOpenGLWidget>
#include <QMouseEvent>
#include <QPointF>
#include <QTime>

// Forward Declarations
class UIControl;
class Renderer;

using namespace std;

class GLWidget : public QOpenGLWidget//, protected QOpenGLFunctions
{
    Q_OBJECT
public:
    explicit GLWidget(QWidget *parent = nullptr);
    virtual ~GLWidget();

    void processTimedEvent(const int x);

    void setUIControl(UIControl* uiControl);

    void setRenderer(Renderer* renderer);

    Eigen::Vector3f getCameraPos() const;

    Eigen::Vector3f getCameraDir() const;

    void updateCameraSpeedAfterKeyPress();

protected:
    virtual void initializeGL();
    virtual void paintGL();
    virtual void resizeGL(int w, int h);

    virtual void keyPressEvent(QKeyEvent* event);
    virtual void keyReleaseEvent(QKeyEvent* event);

    virtual void mousePressEvent(QMouseEvent* event);
    virtual void mouseReleaseEvent(QMouseEvent* event);
    virtual void mouseMoveEvent(QMouseEvent* event);

private:
    void setDefaults();

    // Calculates the camera direction from the angle.
    void angleToDir();

    void updateCameraPos(float stepSize);

    Eigen::Vector3f mCameraPos;
    Eigen::Vector3f mCameraVel;
    Eigen::Vector3f mCameraDir;
    QPointF mMousePos;
    QPointF mAngle; // (x, y) camera angle in degree
    float mMouseSensitivy;
    float mMovementSpeed;

    Renderer* mRenderer;
    UIControl* mUiControl;

    QTime mTime;
    int mMillisInThisSecond;
    int mFramesInThisSecond;

};

#endif // GLWIDGET_H
