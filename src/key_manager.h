#ifndef KEYMANAGER_H
#define KEYMANAGER_H

#include <QKeyEvent>

class KeyManager
{
private:
    KeyManager();

public:
    static void keyDownEvent(QKeyEvent *event);

    static void keyReleaseEvent(QKeyEvent *event);

    static bool isKeyDown(Qt::Key key);

private:
    static bool keyDown[1024];
};

#endif // KEYMANAGER_H
