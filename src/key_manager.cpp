#include "key_manager.h"

#include <QDebug>
#include <QKeySequence>

KeyManager::KeyManager()
{

}


void KeyManager::keyDownEvent(QKeyEvent *event)
{
    //keyDown[event->key()] = true;

//    qDebug() << event->key() << "\n";
//    qDebug() << QKeySequence(event->key()).toString() << " " << event->text() << "\n";
//    if (event->modifiers() & Qt::ShiftModifier)
//        qDebug() << "shift is pressed" << "\n";
//    if (event->modifiers() & Qt::ControlModifier)
//        qDebug() << "ctrl is pressed" << "\n";
}

void KeyManager::keyReleaseEvent(QKeyEvent *event)
{
    //keyDown[event->key()] = false;
}

bool KeyManager::isKeyDown(Qt::Key key)
{
    return keyDown[key];
}



bool KeyManager::keyDown[1024];
